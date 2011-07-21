/*
 * MPN2D.cpp
 *
 *  Created on: Jun 9, 20111
 *      Author: nlacock
 */

#include "MPN2D.h"
#include "Integrator.h"
#include "Environment.h"
#include "specialfunctions.h"
#include "gamma.h"
#include <cmath>
#include <iostream>

double noExtraCost(Environment * e,double * state){return 0.;}

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

double anglePerturb(MPNParams * params, double * gradient,double polytime){
	double legendreSum = 0;
	for(unsigned int i(0); i<params->nLegendrePolys; i++){
	  if(params->controlParameters[i] != 0)
	    legendreSum += params->controlParameters[i]*alglib::legendrecalculate(i,polytime);
	}

	return legendreSum*(M_PI);

}

//Size is the index of the path to use, usually the number of steps
double terminalCost(Environment * e,double ** path,int size){
	//std::cout << "terminalIndex: " << path[size-1][0] << "," << path[size-1][1];
	return e->potentialField(path[size-1]);
}

//Size should be the actual size of path and controlPath
double incrementalCost(Environment * e, MPNParams * params,double ** path, double ** controlPath, double dt, int size, double(*extraCost)(Environment *,double *)){

	//compute the value of the cost function at every point
	double costFunction[size];
	for(int i(0); i<size; i++){
	  costFunction[i] = params->costWeights[0]*gamma(path[i],e->goal,DIM) +
	    params->costWeights[1]*gamma(controlPath[i],DIM) + 
	    params->costWeights[2]*extraCost(e,path[i]);
	}

	//do a cumulative integration of the cost function (wrt time) using the trapezoidal method
	double cost = costFunction[0] + costFunction[size-1];
	for(int i(1); i<size-1; i++){
		cost += 2*costFunction[i];
	}

	return (dt/2.)*cost;
}

//Puts the nominal controlPath and the path in the given pointers
void nominalPath(Environment * e,Integrator * intgr,double** controlPath, double ** path,double * start,int steps){
  double * current = start;
  for(int i(0); i<steps; i++){
    e->negatedGradient(current,controlPath[i]);
    intgr->step(current,controlPath[i],path[i]);
    current = path[i];
  }
}

//Puts a sample controlPath and path into the given variables, given a set of parameters
void samplePath(Environment * e, Integrator * intgr,MPNParams * params,double** controlPath, double ** path,double * start,int steps){
  double * current = start,angPerturb,tmpSin,tmpCos;
  double * currentGrad;
  double currentPolyTime = params->currentTime/params->predictionHorizon;
  double polyDt = intgr->getDt()/params->predictionHorizon;

  for(int i(0); i<steps; i++){
    
    currentGrad = controlPath[i];
    e->negatedGradient(current,currentGrad);//store nominal gradient
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
    //std::cout << "CurrentGrad after: " << currentGrad[0] << "," << currentGrad[1] << std::endl;

    intgr->step(current,currentGrad,path[i]);
    current = path[i];
    
    //std::cout << current[0] << "," << current[1] << std::endl;
  }
}

//TODO: Should also save the state of the integrator and return it
//TODO: Put in a tolerance for the end condition
void generateBestPath(Environment * e, MPNParams * params, Integrator * intgr, double ** &bestPath, double **& bestControl, int & steps, int & controlHorizonIndex,  double * start){
  
  //ln(a) = log(a)/log(e) -> ln(a)/ln(b) = log(a)/log(b)
  double nSamples = log(1/(params->confidence))/log(1/(1-params->level));
  //std::cout << "samples: " << nSamples << std::endl;
  steps = ceil(static_cast<double>(params->predictionHorizon)/intgr->getDt());
  controlHorizonIndex = ceil(static_cast<double>(params->controlHorizon/
						 intgr->getDt()));
  
  if(steps < 1)
    steps = 1;
  if(controlHorizonIndex > steps)
    controlHorizonIndex = steps-1;
  
  double startCost = e->potentialField(start);
  
  double ** nominal = allocatePoints(steps);
  double ** nominalControl = allocatePoints(steps);
  nominalPath(e,intgr,nominalControl,nominal,start,steps);  

  //initialize with nominal values
  Integrator * optimalEndIntegrator = new Integrator(*intgr);
  double ** optimalPath = allocatePoints(steps);
  for(int i(0); i<steps; i++){
    optimalPath[i][0] = nominal[i][0];
    optimalPath[i][1] = nominal[i][1];
  }
  double ** optimalControl = allocatePoints(steps);
  for(int i(0); i<steps; i++){
    optimalControl[i][0] = nominalControl[i][0];
    optimalControl[i][1] = nominalControl[i][1];
  }

  double incrCost = incrementalCost(e,params,nominal,nominalControl,
				    intgr->getDt(),steps);
  double termCost = terminalCost(e,nominal,steps);
  double optimalCost = incrCost + termCost;
  
  
  double optimalParams[params->nLegendrePolys];
  for(unsigned int i(0); i<params->nLegendrePolys;i++){optimalParams[i]=0;}
  
  double currentCost,currentTerminal,currentControlHorizonCost,currentFinalOri;
  double ** currentPath = allocatePoints(steps);
  double ** currentControlPath = allocatePoints(steps);
  
  int acceptedSoFar = 1;
  do{
    intgr->reset();
    //generate control inputs on +/- 1/(number of inputs)
    for(unsigned int i(0); i<params->nLegendrePolys; i++){
      params->controlParameters[i] = (static_cast<double>(rand() - rand())/RAND_MAX)/static_cast<double>(params->nLegendrePolys);
    }
    
    samplePath(e,intgr,params,currentControlPath,currentPath,start,steps);
    currentControlHorizonCost = terminalCost(e,currentPath,controlHorizonIndex);
    
    //If we made progress toward the goal, count this path
    if(currentControlHorizonCost < startCost){
      currentTerminal = terminalCost(e,currentPath,steps);
      
      //Only calculate incremental cost if there is a chance this path is 
      //optimal
      if(currentTerminal < optimalCost){
	currentCost = currentTerminal + incrementalCost(e,params,currentPath,currentControlPath,intgr->getDt(),steps);
	acceptedSoFar++;
	//If we found a better path, save it
	if(currentCost < optimalCost){
	  optimalCost = currentCost;
	  //Copy values into optimalPath
	  for(int i(0); i<steps; i++){
	    optimalPath[i][0] = currentPath[i][0];
	    optimalPath[i][1] = currentPath[i][1];
	  }
	  //Copy values into optimalControl
	  for(int i(0); i<steps; i++){
	    optimalControl[i][0] = currentControlPath[i][0];
	    optimalControl[i][1] = currentControlPath[i][1];
	  }
	  //Copy parameters into optimalParams
	  for(unsigned int i(0); i<params->nLegendrePolys; i++){
	    optimalParams[i] = params->controlParameters[i];
	  }
	  //Copy Integrator end state into optimalEndIntegrator
	  *optimalEndIntegrator = *intgr;
	}
      }
    }
    
  }while(acceptedSoFar < nSamples);
  
  bestPath = optimalPath;
  bestControl = optimalControl;
  for(unsigned int i(0); i<params->nLegendrePolys; i++){
    params->controlParameters[i] = optimalParams[i];
  }

  *intgr = *optimalEndIntegrator;
  intgr->saveState();

  cleanupPoints(nominal,steps);
  cleanupPoints(currentPath,steps);
  cleanupPoints(nominalControl,steps);
  cleanupPoints(currentControlPath,steps);
  delete optimalEndIntegrator;
}
