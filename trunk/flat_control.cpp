#include "flat_control.h"
//#include "traj_control.h"
#include "gamma.h"
#include "saturate.h"
#include <time.h>

#define DEGREE 5

//Controller proportions
#define KPX 1
#define KDX 2
#define KPY 1
#define KDY 2

#define CMD_RATE .3

using namespace alglib;
using namespace PlayerCc;


  //If position reading is done on a different interface from position commands
flat_control::flat_control(Robot * robot, double * endGoal,double tol,
			   double vmax,double omegamax,
			   double vmin,double omegamin){
  this->robot = robot;

  this->endGoal[0] = endGoal[0];
  this->endGoal[1] = endGoal[1];

  this->tol = tol;
  this->vmax = vmax;
  this->wmax = omegamax;
  this->vmin = vmin;
  this->wmin = omegamin;

  /*
  this->xCurrent = new barycentricinterpolant();
  this->yCurrent = new barycentricinterpolant();
  this->xNext = new barycentricinterpolant();
  this->yNext = new barycentricinterpolant();
  */

  this->xCurrent = new spline1dinterpolant();
  this->yCurrent = new spline1dinterpolant();
  this->xNext = new spline1dinterpolant();
  this->yNext = new spline1dinterpolant();

  this->robot->refresh();
  this->pose[0] = this->robot->getX();
  this->pose[1] = this->robot->getY();
  this->pose[2] = this->robot->getYaw();

  this->first = true;
}

void flat_control::preProcess(double time,double ** path, double ** pathDeriv,
			      int steps){
  this->nextDuration = time;
  //Copy the path into arrays
  real_1d_array x,y,t,dx,dy;
  x.setlength(steps+1);
  y.setlength(steps+1);
  dx.setlength(steps+1);
  dy.setlength(steps+1);
  t.setlength(steps+1);

  double dt = time/static_cast<double>(steps);
  for(int i(0); i<steps+1; i++){
    t[i] = (i)*dt;
    x[i] = path[i][0];
    y[i] = path[i][1];
    dx[i] = pathDeriv[i][0];
    dy[i] = pathDeriv[i][1];
  }


  /*
  //put constraints on the endpoint
  tc.setlength(2);
  xc.setlength(2);
  yc.setlength(2);

  tc[0] = time;
  tc[1] = 0;

  xc[0] = path[steps][0];
  xc[1] = path[0][0];

  yc[0] = path[steps][1];
  yc[1] = path[0][1];

  integer_1d_array type("[0,0]");
  int info;
  barycentricfitreport rep;

  barycentricfitfloaterhormannwc(t,x,weights,t.length(),tc,xc,type,tc.length(),DEGREE,info,*this->xNext,rep);
  barycentricfitfloaterhormannwc(t,y,weights,t.length(),tc,yc,type,tc.length(),DEGREE,info,*this->yNext,rep);
  */

  spline1dbuildhermite(t,x,dx,*this->xNext);
  spline1dbuildhermite(t,y,dy,*this->yNext);
  
}

void flat_control::operator()(){

  *this->xCurrent = *this->xNext;
  *this->yCurrent = *this->yNext;
  this->currDuration = this->nextDuration;

  //std::cout << "path = [";
  //this->goal[0] = barycentriccalc(*this->xCurrent,this->currDuration);
  //this->goal[1] = barycentriccalc(*this->yCurrent,this->currDuration);
  this->goal[0] = spline1dcalc(*this->xCurrent,this->currDuration);
  this->goal[1] = spline1dcalc(*this->yCurrent,this->currDuration);
  
  gettimeofday(&this->start,NULL);
  this->startTime = start.tv_sec + start.tv_usec/1000000.;
  this->currTime = this->startTime;
  this->prevTime = this->startTime;
  
  while(this->currTime - this->startTime < this->currDuration 
	&& gamma(this->pose,this->goal,2) > this->tol*this->tol){
    
    gettimeofday(&this->now,NULL);
    this->currTime = this->now.tv_sec + this->now.tv_usec/1000000.;
    
    this->dt = this->currTime - this->prevTime;
    
    /*
    barycentricdiff2(*this->xCurrent,this->currTime-this->startTime,
		     this->desiredX[0],this->desiredV[0],this->desiredA[0]);
    barycentricdiff2(*this->yCurrent,this->currTime-this->startTime,
		     this->desiredX[1],this->desiredV[1],this->desiredA[1]);
    */
    spline1ddiff(*this->xCurrent,this->currTime-this->startTime,
		     this->desiredX[0],this->desiredV[0],this->desiredA[0]);
    spline1ddiff(*this->yCurrent,this->currTime-this->startTime,
		     this->desiredX[1],this->desiredV[1],this->desiredA[1]);

    

    std::cout << desiredX[0] << "," << desiredX[1];
    std::cout << "," << desiredV[0] << "," << desiredV[1];
    std::cout << "," << desiredA[0] << "," << desiredA[1];
    std::cout << "," << this->currTime-this->startTime << std::endl;    
    //std::cout << "desired x: " << desiredX[0] << " desired y: " << desiredX[1] << std::endl;
    
    this->robot->refresh();
    this->pose[0] = this->robot->getX();
    this->pose[1] = this->robot->getY();
    this->pose[2] = this->robot->getYaw();
    //std::cout << pose[0] << "," << pose[1] << ";";
    //if(pose[2] <= 0)
    //pose[2] += 2*M_PI;
    //std::cout << pose[2] << std::endl;
    
    if(gamma(this->pose,this->endGoal,2) < this->tol*this->tol){
      this->robot->setV(0);
      this->robot->setW(0);
      this->robot->send();
      break;
    }
    
    if(this->first){
      //std::cout << "HEREHEREHEREHERE\n";
      this->first = false;
      this->currVel[0] = this->desiredV[0];
      this->currVel[1] = this->desiredV[1];
    }
    else{
      this->currVel[0] = this->robot->getV()*cos(this->pose[2]);
      this->currVel[1] = this->robot->getV()*sin(this->pose[2]);
      //currVel[0] = prevVel*cos(pose[2])*((rand()-rand())
      //				   /static_cast<double>(RAND_MAX));
      //currVel[1] = prevVel*sin(pose[2])*((rand()-rand())
      //				   /static_cast<double>(RAND_MAX));
      
    }
    
    //std::cout << "currvel: " <<currVel[0] << "," << currVel[1] << std::endl;
    
    this->u[0] = this->desiredA[0] + KPX*(this->desiredX[0] - this->pose[0] ) +
      KDX*(this->desiredV[0] - this->currVel[0]);
    this->u[1] = this->desiredA[1] + KPX*(this->desiredX[1] - this->pose[1] ) +
      KDX*(this->desiredV[1] - this->currVel[1]);
    
    //std::cout << "u: " <<u[0] << "," << u[1] << std::endl;
    
    this->acc = this->u[0]*cos(this->pose[2]) + this->u[1]*sin(this->pose[2]);
    v = sqrt(gamma(this->currVel,2)) + this->acc*this->dt;
    omega = (this->u[1]*cos(this->pose[2])-this->u[0]*sin(this->pose[2]))/
      (this->v);
    
    //std::cout << currVel[0] << "," << currVel[1] << std::endl;
    //std::cout << sin(pose[2]) << std::endl;
    
    
    saturate(&this->v,this->vmax,this->vmin);
    //Positive velocities only, thank you very much.
    this->v = this->v > 0 ? this->v : 0;
    saturate(&this->omega,this->wmax,this->wmin);
    
    this->robot->setV(this->v);
    this->robot->setW(this->omega);
    this->robot->send();
    
    //std::cout << "vel: " << v << " omega: " << omega << std::endl;    

    this->prevTime = this->currTime;
    do{
      usleep(200);
      gettimeofday(&this->now,NULL);
      this->currTime = this->now.tv_sec + this->now.tv_usec/1000000.;
    }while(this->currTime-this->prevTime < CMD_RATE);
    
  }  
  //std::cout << "DONE!\n";
  //std::cout << "Time used: " << currTime - startTime << std::endl;
  //std::cout << "];\n";
}

