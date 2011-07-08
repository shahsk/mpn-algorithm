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

#define ROBOTIP "192.168.1.108"
#define ROBOTPORT 6666

#define TOLERANCE .05
#define PRECISION .01
#define DEGREE 3

//Controller proportions
#define KPX 1
#define KDX 2
#define KPY 1
#define KDY 2

#define VMAX .3
#define VMIN .1
#define OMEGAMAX 1
#define OMEGAMIN .5

#define POLYTIME 10

using namespace PlayerCc;


double saturate(double val, double min, double max){
  if(fabs(val) > min && fabs(val) < max)
    return val;

  if(fabs(val) < min){
    if(val > 0)
      return min;
    else
      return -min;
  }
  
  if(fabs(val) > max){
    if(val > 0)
      return max;
    else
      return -max;
  }

}

double satv(double v){return saturate(v,VMIN,VMAX); }
double satw(double w){return saturate(w,OMEGAMIN,OMEGAMAX); }

int sign(double a){
  if(a>0)
    return 1;
  else
    return -1;
}

//Builds 2 polynomials x(t) and y(t) on the given x,y trajectory. t ranges from 0 to 1
void buildPolynoms(double ** path,double ** pathDeriv,int steps,double range,
		  alglib::barycentricinterpolant * xpoly,
		  alglib::barycentricinterpolant * ypoly){

  //Copy the path into arrays
  alglib::real_1d_array x,y,t,dx,dy,weights("[1,1]");
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

  int info;
  alglib::polynomialfitreport junk;
  alglib::integer_1d_array ones("[1,1]");

  /*
alglib::polynomialfitwc(
    real_1d_array x,
    real_1d_array y,
    real_1d_array w,
    real_1d_array xc,
    real_1d_array yc,
    integer_1d_array dc,
    ae_int_t m,
    ae_int_t& info,
    barycentricinterpolant& p,
    polynomialfitreport& rep);
  */
  alglib::polynomialfitwc(t,x,weights,t,dx,ones,DEGREE,info,*xpoly,junk);
  alglib::polynomialfitwc(t,y,weights,t,dy,ones,DEGREE,info,*ypoly,junk);
  
}

struct trajController{
  PlayerClient * client;
  Position2dProxy * pos;

  double goal[2],pose[3],currVel[2],tol,desiredV[2],desiredX[2],desiredA[2],u[2],v,omega,acc,dt;
  int steps;

  timeval start,now;
  double startTime,currTime,prevTime;

  double prevVel,prevOmega;
  trajController(){
    client = new PlayerClient(ROBOTIP,ROBOTPORT);
    pos = new Position2dProxy(client,0);
    tol = TOLERANCE;

    client->Read();
    pose[0] = pos->GetXPos();
    pose[1] = pos->GetYPos();
    pose[3] = pos->GetYaw();

  }

  void operator()(alglib::barycentricinterpolant * xpath,
		  alglib::barycentricinterpolant * ypath,
		  double time,bool correct){

    goal[0] = barycentriccalc(*xpath,time);
    goal[1] = barycentriccalc(*ypath,time);

    steps = ceil(time/PRECISION);
    dt = time/steps;

    gettimeofday(&start,NULL);
    startTime = start.tv_sec + start.tv_usec/1000000.;
    currTime = startTime;
    prevTime = startTime;
    int i=0;

    while(currTime - startTime < time && gamma(pose,goal) > tol*tol ){
    //while(gamma(pose,goal) > tol*tol){
      dt = currTime - prevTime;
      //spline1ddiff(*xpath,currTime-startTime,desiredX[0],desiredV[0],desiredA[0]);
      //spline1ddiff(*ypath,currTime-startTime,desiredX[1],desiredV[1],desiredA[1]);

      barycentricdiff2(*xpath,currTime-startTime,desiredX[0],desiredV[0],desiredA[0]);
      barycentricdiff2(*ypath,currTime-startTime,desiredX[1],desiredV[1],desiredA[1]);

      //std::cout << "desired vx: " << desiredV[0] << " desired vy: " << desiredV[1] << std::endl;
    
      client->Read();
      pose[0] = pos->GetXPos();
      pose[1] = pos->GetYPos();
      pose[2] = pos->GetYaw();

      if(i==0){
	i=-1;
	//if(correct){
	  
	  //do{
	  //omega = satw(atan2(desiredV[1],desiredV[0])-pose[2]);
	  //pos->SetSpeed(0, omega);
	  //client->Read();
	  //pose[0] = pos->GetXPos();
	  //pose[1] = pos->GetYPos();
	  //pose[2] = pos->GetYaw();
	    //}while(omega > tol);
	  //pos->SetSpeed(satv(sqrt(gamma(desiredV))),0);
	//}
	currVel[0] = desiredV[0];
	currVel[1] = desiredV[1];
      }
      else{
	currVel[0] = pos->GetXSpeed()*cos(pose[2]);
	currVel[1] = pos->GetXSpeed()*sin(pose[2]);
	//currVel[0] = prevVel*cos(pose[2])*((rand()-rand())
	//				   /static_cast<double>(RAND_MAX));
	//currVel[1] = prevVel*sin(pose[2])*((rand()-rand())
	//				   /static_cast<double>(RAND_MAX));

      }

      u[0] = desiredA[0] + KPX*(desiredX[0] - pose[0] ) + KDX*(desiredV[0] - currVel[0]);
      u[1] = desiredA[1] + KPX*(desiredX[1] - pose[1] ) + KDX*(desiredV[1] - currVel[1]);
  
      acc = u[0]*cos(pose[2]) + u[1]*sin(pose[2]);
      v = sqrt(gamma(currVel)) + acc*dt;
      omega = (u[1]*cos(pose[2]) - u[0]*sin(pose[2]))/(v);

      v = satv(v);
      omega = satw(omega);

      //std::cout << "vel: " << v << " omega: " << omega << std::endl;
      pos->SetSpeed(v,omega);

      prevTime = currTime;
      gettimeofday(&now,NULL);
      currTime = now.tv_sec + now.tv_usec/1000000.;

      prevVel = v;
      prevOmega = omega;
    }
    

    std::cout << "DONE!\n";
    std::cout << "Time used: " << currTime - startTime << std::endl;
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
  double startOri,finalOri,start[2],dt = PRECISION;//,time=params->controlHorizon;
  double time = POLYTIME;
  double ** bestPath,**bestControl,**nextPath,**nextControl;

  robot.client->Read();
  start[0] = robot.pos->GetXPos();
  start[1] = robot.pos->GetYPos();
  startOri = robot.pos->GetYaw();

  alglib::barycentricinterpolant *currXPoly,*currYPoly,*nextXPoly,*nextYPoly,*tmp;
  currXPoly = new alglib::barycentricinterpolant();
  currYPoly = new alglib::barycentricinterpolant();
  nextXPoly = new alglib::barycentricinterpolant();
  nextYPoly = new alglib::barycentricinterpolant();
  
  
  //Generate an initial path
  generateBestPath(*env,*params,bestPath,bestControl,steps,CHI,start,startOri,dt,finalOri);
  buildPolynoms(bestPath,bestControl,CHI,time,currXPoly,currYPoly);

  std::cout << "displacement: " << sqrt(gamma(bestPath[0],bestPath[CHI])) << std::endl;

  bool dummy = true;
  while(gamma(start,env->goal) > TOLERANCE*TOLERANCE ){

    start[0] = bestPath[CHI][0];
    start[1] = bestPath[CHI][1];

    //start[0] = spline1dcalc(*currXPoly,time); //Save a copy
    //start[1] = spline1dcalc(*currYPoly,time);//bestPath[CHI][1];
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
    
    //params->currentTime += params->controlHorizon;
    //if(params->currentTime > params->predictionHorizon)
    //  params->currentTime = 0;

    //Wait for robot to finish
    driveThread.join();
    dummy = false;

  }  

  robot.pos->SetSpeed(0,0);
  sleep(3);
}
