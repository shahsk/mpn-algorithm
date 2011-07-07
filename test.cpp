#include "matlabTests/configure.h"
#include "alglib/interpolation.h"
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
#define PRECISION .05
#define DEGREE 5

//Controller proportions
#define KPX 2
#define KDX 4
#define KPY 2
#define KDY 4

#define VMAX 1
#define OMEGAMAX 1

#define POLYTIME 40

using namespace PlayerCc;

int sign(double a){
  if(a>0)
    return 1;
  else
    return -1;
}

//Builds 2 polynomials x(t) and y(t) on the given x,y trajectory. t ranges from 0 to 1
void buildPolynoms(double ** path,double ** pathDeriv,int steps,double range,
		  alglib::spline1dinterpolant * xpoly,
		  alglib::spline1dinterpolant * ypoly){

  //Copy the path into arrays
  alglib::real_1d_array x,y,t,dx,dy;
  x.setlength(2);
  y.setlength(2);
  t.setlength(2);
  dx.setlength(2);
  dy.setlength(2);

  x[0] = path[0][0];
  x[1] = path[steps][0];

  y[0] = path[0][1];
  y[1] = path[steps][1];

  dx[0] = (path[1][0]-x[0])/(range/steps);
  dx[1] = (path[steps+1][0]-x[1])/(range/steps);

  dy[0] = (path[1][1]-y[0])/(range/steps);
  dy[1] = (path[steps+1][1]-y[1])/(range/steps);

  t[0] = 0;
  t[1] = range;
  
  /*
  for(int i(0); i<steps; i++){
    x[i] = path[i][0];
    y[i] = path[i][1];
    dx[i] = pathDeriv[i][0];
    dy[i] = pathDeriv[i][1];
    t[i] = (static_cast<double>(i)/static_cast<double>(steps))*range;
    }*/

  alglib::spline1dbuildhermite(t,x,dx,2,*xpoly);
  alglib::spline1dbuildhermite(t,y,dy,2,*ypoly);

}

struct trajController{
  PlayerClient * client;
  Position2dProxy * pos;

  double goal[2],pose[3],currVel[2],tol,desiredV[2],desiredX[2],desiredA[2],u[2],v,omega,acc,dt;
  int steps;

  timeval start,now;
  double startTime,currTime,prevTime;

  trajController(){
    client = new PlayerClient(ROBOTIP,ROBOTPORT);
    pos = new Position2dProxy(client,0);
    tol = TOLERANCE;

    client->Read();
    pose[0] = pos->GetXPos();
    pose[1] = pos->GetYPos();
    pose[3] = pos->GetYaw();

  }

  void operator()(alglib::spline1dinterpolant * xpath,
		  alglib::spline1dinterpolant * ypath,
		  double time,bool correct){

    goal[0] = spline1dcalc(*xpath,time);
    goal[1] = spline1dcalc(*ypath,time);

    steps = ceil(time/.05);
    dt = time/steps;

    gettimeofday(&start,NULL);
    startTime = start.tv_sec + start.tv_usec/1000000.;
    currTime = startTime;
    prevTime = startTime;
    int i=0;
    while(currTime - startTime < time){
      dt = currTime - prevTime;
      spline1ddiff(*xpath,currTime-startTime,desiredX[0],desiredV[0],desiredA[0]);
      spline1ddiff(*ypath,currTime-startTime,desiredX[1],desiredV[1],desiredA[1]);

      //std::cout << "desired vx: " << desiredV[0] << " desired vy: " << desiredV[1] << std::endl;
    
      client->Read();
      pose[0] = pos->GetXPos();
      pose[1] = pos->GetYPos();
      pose[2] = pos->GetYaw();

      if(i==0){
	i=-1;
	if(correct){
	  pose[2] = atan2(desiredV[1],desiredV[0]);
	  pos->GoTo(pose[0],pose[1],pose[2]);
	  sleep(3);
	}
	currVel[0] = desiredV[0];
	currVel[1] = desiredV[1];
      }
      else{
	currVel[0] = pos->GetXSpeed()*cos(pose[2]);
	currVel[1] = pos->GetXSpeed()*sin(pose[2]);
      }

      u[0] = desiredA[0] + KPX*(desiredX[0] - pose[0] ) + KDX*(desiredV[0] - currVel[0]);
      u[1] = desiredA[1] + KPX*(desiredX[1] - pose[1] ) + KDX*(desiredV[1] - currVel[1]);
  
      acc = u[0]*cos(pose[2]) + u[1]*sin(pose[2]);
      v = fabs(pos->GetXSpeed()) + acc*dt;
      omega = (u[1]*cos(pose[2]) - u[0]*sin(pose[2]))/(v);

      //Saturate omega
      if(omega > 0 && omega > OMEGAMAX)
	omega = OMEGAMAX;
      if(omega < 0 && omega < -OMEGAMAX)
	omega = -OMEGAMAX;

      //Saturate v
      if(v > 0 && v > v)
	v = VMAX;
      if(v < 0 && v < -VMAX)
	v = -VMAX;


      //std::cout << "vel: " << v << " omega: " << omega << std::endl;
      pos->SetSpeed(v,omega);

      prevTime = currTime;
      gettimeofday(&now,NULL);
      currTime = now.tv_sec + now.tv_usec/1000000.;
    }
      std::cout << "DONE!\n";
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
  double startOri,finalOri,start[2],dt = PRECISION,time=params->controlHorizon;
  double ** bestPath,**bestControl,**nextPath,**nextControl;

  robot.client->Read();
  start[0] = robot.pos->GetXPos();
  start[1] = robot.pos->GetYPos();
  startOri = robot.pos->GetYaw();

  alglib::spline1dinterpolant *currXPoly,*currYPoly,*nextXPoly,*nextYPoly,*tmp;
  currXPoly = new alglib::spline1dinterpolant();
  currYPoly = new alglib::spline1dinterpolant();
  nextXPoly = new alglib::spline1dinterpolant();
  nextYPoly = new alglib::spline1dinterpolant();
  
  
  //Generate an initial path
  generateBestPath(*env,*params,bestPath,bestControl,steps,CHI,start,startOri,dt,finalOri);
  buildPolynoms(bestPath,bestControl,CHI,time,currXPoly,currYPoly);

  std::cout << "displacement: " << sqrt(gamma(bestPath[0],bestPath[CHI])) << std::endl;

  bool dummy = true;
  while(gamma(start,env->goal) > TOLERANCE*TOLERANCE ){

    start[0] = bestPath[CHI][0];//spline1dcalc(*currXPoly,time); //Save a copy
    start[1] = bestPath[CHI][1];//spline1dcalc(*currYPoly,time);//bestPath[CHI][1];
    //std::cout << start[0] << "," << start[1] << std::endl;
    
    //Start driving along the path
    boost::thread driveThread(robot,currXPoly,currYPoly,time,dummy);
    
    //Compute the next path
    startOri = finalOri;
    generateBestPath(*env,*params,nextPath,nextControl,nextSteps,nextCHI,start,startOri,dt,finalOri);
    buildPolynoms(nextPath,nextControl,nextCHI,time,nextXPoly,nextYPoly);

    std::cout << "nominal final pos: " << nextPath[nextCHI][0] << "," << nextPath[nextCHI][1] << std::endl;
    std::cout << "displacement: " << sqrt(gamma(nextPath[0],nextPath[nextCHI])) << std::endl;

    //Set up for next iteration
    cleanupPoints(bestPath,steps);
    cleanupPoints(bestControl,steps);
    bestPath = nextPath;
    bestControl = nextControl;
    steps = nextSteps;
    CHI = nextCHI;

    //pointer swap
    tmp = currXPoly;
    currXPoly = nextXPoly;
    nextXPoly = tmp;

    tmp = currYPoly;
    currYPoly = nextYPoly;
    nextYPoly = tmp;
    
    params->currentTime += params->controlHorizon;
    if(params->currentTime > params->predictionHorizon)
      params->currentTime = 0;

    //Wait for robot to finish
    driveThread.join();
    dummy = false;

  }  
  
}
