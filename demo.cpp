#include "MPN2D.h"
#include "Integrator.h"
#include "Build.h"
#include "Environment.h"
#include "Obstacle.h"
#include "gamma.h"
#include "saturate.h"
#include <math.h>
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <iostream>
#include <time.h>
#include <libconfig.h++>
#include <string>

#include <boost/thread.hpp>

#include "flat_control.h"

#define CONFIG_FILE "lab.cfg"

#define ROBOTIP "localhost"//"192.168.1.108"
#define ROBOTPORT 6665
#define SOURCEIP "localhost"
#define SOURCEPORT 6665

#define USE_ROBOT_POS true

#define PRECISION .01

//-1 means unconstrained
#define VMAX .3
#define VMIN .1
#define OMEGAMAX .5
#define OMEGAMIN -1

using namespace std;
using namespace PlayerCc;

int main(){
  //Setup
  char filename[] = CONFIG_FILE;
  Environment * env;
  MPNParams * params;
  Integrator * intgr;

  libconfig::Config c;
  c.readFile(filename);

  buildEnvironment(&c,env,2);
  buildMPNParams(&c,params);

  PlayerClient *sourceC,*sinkC = new PlayerClient(ROBOTIP,ROBOTPORT);
  Position2dProxy *sourceP,*sinkP = new Position2dProxy(sinkC,0);
  if(USE_ROBOT_POS){
    sourceC = sinkC;
    sourceP = sinkP;
  }
  else{
    sourceC = new PlayerClient(SOURCEIP,SOURCEPORT);
    sourceP = new Position2dProxy(sourceC,0);
  }

  flat_control robot(sourceC,sourceP,env->goal,params->tolerance,
		     sinkC,sinkP,VMAX,OMEGAMAX,VMIN,OMEGAMIN);

  int steps,CHI,nextSteps,nextCHI; //CHI = Control Horizon Index
  double start[2],dt = PRECISION;//,time=params->controlHorizon;
  double time = params->controlHorizon;
  double ** bestPath,**bestControl,**nextPath,**nextControl;

  sourceC->Read();
  start[0] = sourceP->GetXPos();
  start[1] = sourceP->GetYPos();

  intgr = new Unicycle(dt,sourceP->GetYaw(),VMAX,OMEGAMAX,VMIN,OMEGAMIN);
  
  //Generate an initial path
  bool done = generateBestPath(env,params,intgr,bestPath,bestControl,
			       steps,CHI,start);
  robot.preProcess(time,bestPath,bestControl,CHI);

  double realPos[2] = {start[0],start[1]};
  while(!done){

    start[0] = bestPath[CHI][0];
    start[1] = bestPath[CHI][1];

    boost::thread driveThread(robot);

    //Compute the next path
    done = generateBestPath(env,params,intgr,nextPath,nextControl,nextSteps,nextCHI,start);
    //buildPolynoms(nextPath,nextControl,nextCHI,time,nextXPoly,nextYPoly);
    robot.preProcess(time,nextPath,nextControl,nextCHI);

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

    //Wait for robot to finish
    driveThread.join();
    realPos[0] = sourceP->GetXPos();
    realPos[1] = sourceP->GetYPos();

  }  

  //Drive the final stretch
  boost::thread driveThread(robot);
  driveThread.join();
  sinkP->SetSpeed(0,0);
  sleep(3);
}
