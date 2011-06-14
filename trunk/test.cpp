#include "MPN2D.h"
#include "Environment.h"
#include "Obstacle.h"
#include <iostream>

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


	for(int i(0); i<s; i++){
		std::cout << path[i][0] << "," << path[i][1] << std::endl;
	}

	//samplePath(e,params,controlPath,path,start,dt,steps);

}
