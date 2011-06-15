#include "MPN2D.h"
#include "Environment.h"
#include "Obstacle.h"
#include <iostream>

int main(){
	double goal[2] = {0,0};
	Environment e(goal,3.0,10.0);

	//Obstacle setup
	double obPos1[2] = {-3.0,-1.0};
	double obPos2[2] = {-2.0,-2.0};
	e.obstacles.push_back(Obstacle(obPos1,.2));
	e.obstacles.push_back(Obstacle(obPos2,.2));

	MPNParams params;
	double controlParams[5] = {0,0,0,0,0};
	params.controlParameters = controlParams;
	params.nLegendrePolys = 5;
	params.currentTime = 32.0;
	params.predictionHorizon = 100.0;
	params.controlHorizon = 25.0;
	params.confidence = .05;
	params.level = .05;

	double start[2] = {-0.524105140444470,-2.046514982759156};
	//int steps = 100;
	double ** controlPath; //= allocatePoints(steps);
	double ** path; //= allocatePoints(steps);
	double dt = .1;
	int s,controlIndex;

	generateBestPath(e, params, path, controlPath, s, controlIndex,
				 start, dt);


	for(int i(0); i<s; i++){
	  std::cout << path[i][0] << "," << path[i][1] << std::endl;
	  if(i == controlIndex){
	    std::cin.get();
	  }
	}

	//samplePath(e,params,controlPath,path,start,dt,steps);

}
