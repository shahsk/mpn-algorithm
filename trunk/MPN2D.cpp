/*
 * MPN2D.cpp
 *
 *  Created on: Jun 9, 20111
 *      Author: nlacock
 */

#include "MPN2D.h"
#include "Environment.h"
#include "src/specialfunctions.h"
#include <cmath>
#include <iostream>

double noExtraCost(Environment & e,double * state){return 0.;}

//Allocates a given number of double[2] points
double ** allocatePoints(int npoints){
	double ** output = new double*[npoints];
	for(int i(0); i<npoints; i++)
		output[i] = new double[2];
	return output;
}

//Deallocates a given number of double[2] points, meant to be used in conjunction with allocatePoints
void cleanupPoints(double ** pointArray,int npoints){
	for(int i(0); i<npoints; i++)
		delete [] pointArray[i];
	delete [] pointArray;
}

//Puts the nominal controlPath and the path in the given pointers
void nominalPath(Environment & e,double** controlPath, double ** path,double * start, double dt, int steps){
	double * current = start;
	for(int i(0); i<steps; i++){
		e.negatedGradient(current,controlPath[i]);
		path[i][0] = current[0] + controlPath[i][0]*dt;
		path[i][1] = current[1] + controlPath[i][1]*dt;
		current = path[i];
	}
}

//Puts a sample controlPath and path into the given variables, given a set of parameters
void samplePath(Environment & e, MPNParams & params,double** controlPath, double ** path,double * start, double dt, int steps){

	double * current = start,angPerturb,tmpSin,tmpCos;
	double * currentGrad;
	double currentPolyTime = params.currentTime/params.predictionHorizon;
	double polyDt = dt/params.predictionHorizon;


	for(int i(0); i<steps; i++){

		currentGrad = controlPath[i];
		e.negatedGradient(current,currentGrad);//store nominal gradient
		//std::cout << "Current: " << current[0] << "," << current[1] << std::endl;
		//std::cout << "CurrentGrad: " << currentGrad[0] << "," << currentGrad[1] << std::endl;
		currentPolyTime += polyDt;
		if(i != 0){//perturb the angle of the negated gradient, except for the first step
			angPerturb = anglePerturb(params,currentGrad,currentPolyTime);
			//std::cout << "angPerturb: " << angPerturb << std::endl;
			tmpCos = cos(angPerturb);
			tmpSin = sin(angPerturb);

			//store new gradient
			currentGrad[0] = currentGrad[0]*tmpCos - currentGrad[1]*tmpSin;
			currentGrad[1] = currentGrad[1]*tmpCos + currentGrad[0]*tmpSin;
		}
		path[i][0] = current[0] + currentGrad[0]*dt;
		path[i][1] = current[1] + currentGrad[1]*dt;

		current = path[i];

		//std::cout << current[0] << "," << current[1] << std::endl;
	}
}

double anglePerturb(MPNParams & params, double * gradient,double polytime){
	double legendreSum = 0;
	for(unsigned int i(0); i<params.nLegendrePolys; i++){
		legendreSum += params.controlParameters[i]*alglib::legendrecalculate(i,polytime);
	}

	legendreSum*=(M_PI/2.);
	return atan2(gradient[1]+legendreSum,gradient[0]+legendreSum);
}

//Size is the index of the path to use, usually the number of steps
double terminalCost(Environment & e,double ** path,int size){
	//std::cout << "terminalIndex: " << path[size-1][0] << "," << path[size-1][1];
	return e.potentialField(path[size-1]);
}

//Size should be the actual size of path and controlPath
double incrementalCost(Environment & e, MPNParams & params,double ** path, double ** controlPath, double dt, int size, double(*extraCost)(Environment &,double *)){

	//compute the value of the cost function at every point
	double costFunction[size];
	for(int i(0); i<size; i++){
		costFunction[i] = params.costWeights[0]*gamma(path[i],e.goal) +
				params.costWeights[1]*gamma(controlPath[i]) + params.costWeights[2]*extraCost(e,path[i]);
	}

	//do a cumulative integration of the cost function (wrt time) using the trapezoidal method
	double cost = costFunction[0] + costFunction[size-1];
	for(int i(1); i<size-1; i++){
		cost += 2*costFunction[i];
	}

	return (dt/2.)*cost;
}


int generateBestPath(Environment & e, MPNParams & params, double ** &bestPath,double * start, double dt){

	//ln(a) = log(a)/log(e) -> ln(a)/ln(b) = log(a)/log(b)
	double nSamples = log(1/(params.confidence))/log(1/(1-params.level));
	std::cout << "samples: " << nSamples << std::endl;
	int steps = ceil(static_cast<double>(params.predictionHorizon - params.currentTime)/dt);
	int controlHorizonIndex = ceil(static_cast<double>(params.controlHorizon + params.currentTime)/dt);

	double startCost = e.potentialField(start);

	double ** nominal = allocatePoints(steps);
	double ** nominalControl = allocatePoints(steps);

	nominalPath(e,nominalControl, nominal, start, dt, steps);

	//initialize with nominal values
	double ** optimalPath = allocatePoints(steps);
	for(int i(0); i<steps; i++){
		optimalPath[i][0] = nominal[i][0];
		optimalPath[i][1] = nominal[i][1];
	}

	double incrCost = incrementalCost(e,params,nominal,nominalControl,dt,steps);
	double termCost = terminalCost(e,nominal,steps);

	double optimalCost = incrCost + termCost;
	//double optimalCost = terminalCost(e,nominal,steps) + incrementalCost(e,params,nominal,nominalControl,dt,steps);


	double optimalParams[params.nLegendrePolys];
	for(unsigned int i(0); i<params.nLegendrePolys; i++){optimalParams[i] = params.controlParameters[i];}

	double currentCost,currentTerminal,currentControlHorizonCost;
	double ** currentPath = allocatePoints(steps);
	double ** currentControlPath = allocatePoints(steps);

	int acceptedSoFar = 1;
	do{

		//generate control inputs on +/- 1/(number of inputs)
		for(unsigned int i(0); i<params.nLegendrePolys; i++){
			params.controlParameters[i] = (static_cast<double>(rand() - rand())/RAND_MAX)/static_cast<double>(params.nLegendrePolys);
		}

		samplePath(e,params,currentControlPath,currentPath,start,dt,steps);
		currentControlHorizonCost = terminalCost(e,currentPath,controlHorizonIndex);
		//If we made progress toward the goal, count this path
		if(currentControlHorizonCost < startCost){
			currentTerminal = terminalCost(e,currentPath,steps);
			currentCost = currentTerminal + incrementalCost(e,params,currentPath,currentControlPath,dt,steps);
			acceptedSoFar++;
			//If we found a better path, save it
			if(currentCost < optimalCost){
				optimalCost = currentCost;
				//Copy values into optimalPath
				for(int i(0); i<steps; i++){
					optimalPath[i][0] = currentPath[i][0];
					optimalPath[i][1] = currentPath[i][1];
				}
				//Copy parameters into optimalParams
				for(unsigned int i(0); i<params.nLegendrePolys; i++){
					optimalParams[i] = params.controlParameters[i];
				}
			}
		}

	}while(acceptedSoFar < nSamples);

	bestPath = optimalPath;
	for(unsigned int i(0); i<params.nLegendrePolys; i++){
		params.controlParameters[i] = optimalParams[i];
	}

	cleanupPoints(nominal,steps);
	cleanupPoints(currentPath,steps);
	cleanupPoints(nominalControl,steps);
	cleanupPoints(currentControlPath,steps);

	return steps;
}

int main(){
	double goal[2] = {.5,.5};
	Environment e(goal,1.3,1.0);

	//Obstacle setup
	double obPos1[2] = {.1,.1};
	double obPos2[2] = {.1,.3};
	e.obstacles.push_back(Obstacle(obPos1,.2));
	e.obstacles.push_back(Obstacle(obPos2,.2));

	MPNParams params;
	double controlParams[5] = {0,0,0,0,0};
	params.controlParameters = controlParams;
	params.nLegendrePolys = 5;
	params.currentTime = 0.0;
	params.predictionHorizon = 1.0;
	params.controlHorizon = .25;
	params.confidence = .05;
	params.level = .05;

	double start[2] = {.4,.3};
	int steps = 100;
	//double ** controlPath = allocatePoints(steps);
	double ** path; //= allocatePoints(steps);
	double dt = .01;

	int s = generateBestPath(e, params, path, start, dt);

	/*
	for(int i(0); i<s; i++){
		std::cout << path[i][0] << "," << path[i][1] << std::endl;
	}
	*/
	//samplePath(e,params,controlPath,path,start,dt,steps);

}
