#include "matlabTests/configure.h"
#include "MPN2D.h"
#include "Environment.h"
#include "Obstacle.h"
#include <math.h>
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <iostream>
#include <time.h>

#include <boost/thread.hpp>

#define ROBOTIP "localhost"
#define ROBOTPORT 6665

#define TOLERANCE .1
#define PRECISION .01

using namespace PlayerCc;

struct trajController{
  PlayerClient * client;
  Position2dProxy * pos;

  double tmp[3],tol;

  trajController(){
    client = new PlayerClient(ROBOTIP,ROBOTPORT);
    pos = new Position2dProxy(client,0);
    tol = TOLERANCE;

    client->Read();
    tmp[0] = pos->GetXPos();
    tmp[1] = pos->GetYPos();
    tmp[3] = pos->GetYaw();
  }

  void operator()(double ** path,double ** control,int CHI){
    //Complicated stuff happens to drive the robot

    //... but for now, we have this
    pos->GoTo(path[CHI][0],path[CHI][1],
	      atan2(control[CHI][1],control[CHI][0]) );
    do{
      sleep(.1);
      client->Read();
      tmp[0] = pos->GetXPos();
      tmp[1] = pos->GetYPos();
      tmp[3] = pos->GetYaw();
    }while(gamma(tmp,path[CHI]) > tol*tol );
    
    return;
  }

};

int main(){
  
  //Setup
  char filename[] = "lab.cfg";
  Environment * env;
  MPNParams * params;
  
  configure(filename,env,params);

  trajController robot;


  int steps,CHI,nextSteps,nextCHI; //CHI = Control Horizon Index
  double startOri,finalOri,start[2],dt = PRECISION;
  double ** bestPath,**bestControl,**nextPath,**nextControl;

  robot.client->Read();
  start[0] = robot.pos->GetXPos();
  start[1] = robot.pos->GetYPos();
  startOri = robot.pos->GetYaw();
  
  //Generate an initial path
  generateBestPath(*env,*params,bestPath,bestControl,steps,CHI,start,startOri,dt,finalOri);

  while(gamma(start,env->goal) > TOLERANCE*TOLERANCE ){

    start[0] = bestPath[CHI][0]; //Save a copy
    start[1] = bestPath[CHI][1];
    std::cout << start[0] << "," << start[1] << std::endl;
    
    //Start driving along the path
    boost::thread driveThread(robot,bestPath,bestControl,CHI);
    
    //Compute the next path
    startOri = finalOri;
    generateBestPath(*env,*params,nextPath,nextControl,nextSteps,nextCHI,start,startOri,dt,finalOri);
    
    //Wait for robot to finish
    driveThread.join();

    //Set up for next iteration
    cleanupPoints(bestPath,steps);
    cleanupPoints(bestControl,steps);
    bestPath = nextPath;
    bestControl = nextControl;
    steps = nextSteps;
    CHI = nextCHI;
    
    params->currentTime += params->controlHorizon;
    if(params->currentTime > params->predictionHorizon)
      params->currentTime = 0;

  }  
  
}
