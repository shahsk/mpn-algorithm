#include "MPN2D.h"
#include "Integrator.h"
#include "Build.h"
#include "Environment.h"
#include "Obstacle.h"
#include "gamma.h"
#include "saturate.h"
#include "flat_control.h"

#include "Robot.h"
#include "PlayerRobot.h"

#include <math.h>
#include <libplayerc++/playerc++.h>
#include <iostream>
#include <time.h>
#include <libconfig.h++>
#include <string>
#include <boost/thread.hpp>

#define CONFIG_FILE "lab.cfg"

#define ROBOTIP "localhost"
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

  Robot * interface = new PlayerRobot(sourceC,sourceP,sinkC,sinkP); 
  //PlayerRobot * interface = new PlayerRobot(sourceC,sourceP,sinkC,sinkP);

  flat_control robot(interface, env->goal,params->tolerance,
	       VMAX,OMEGAMAX,VMIN,OMEGAMIN);
  //flat_control robot(sourceC,sourceP,env->goal,params->tolerance,
  //		     sinkC,sinkP,VMAX,OMEGAMAX,VMIN,OMEGAMIN);

  int steps,CHI,nextSteps,nextCHI; //CHI = Control Horizon Index
  double start[2],dt = PRECISION;//,time=params->controlHorizon;
  double time = params->controlHorizon;
  double ** bestPath,**bestControl,**nextPath,**nextControl;

  interface->refresh();
  start[0] = interface->getX();
  start[1] = interface->getY();

  intgr = new Unicycle(dt,interface->getYaw(),VMAX,OMEGAMAX,VMIN,OMEGAMIN);
  
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
    interface->refresh();
    realPos[0] = interface->getX();
    realPos[1] = interface->getY();

  }  

  //Drive the final stretch
  boost::thread driveThread(robot);
  driveThread.join();
  interface->setV(0);
  interface->setW(0);
  interface->send();
  sleep(3);
}
