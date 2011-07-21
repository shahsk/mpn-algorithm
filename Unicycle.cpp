
#include "Unicycle.h"
#include "math.h"

Unicycle::Unicycle(double stepTime,double theta,double vmax,double omegamax,
		   double vmin,double omegamin): Integrator(stepTime,2){
  this->startTheta = theta;
  this->vmax = vmax;
  this->omegamax = omegamax;
  this->vmin = vmin;
  this->omegamin = omegamin;

  this->currTheta = theta;

}

Unicycle::Unicycle(libconfig::Setting& group,double stepTime,unsigned int dim):
  Integrator(stepTime,2){
  
  this->startTheta = group["start_orientation"];
  this->currTheta = this->startTheta;

  this->vmax = -1;
  this->vmin = -1;
  this->omegamax = -1;
  this->omegamin = -1;

  group.lookupValue("max_velocity",this->vmax);
  group.lookupValue("min_velocity",this->vmin);
  group.lookupValue("max_angular_velocity",this->omegamax);
  group.lookupValue("min_angular_velocity",this->omegamin);

}

void Unicycle::reset(){
  this->currTheta = this->startTheta;
}

double Unicycle::sign(double num){
  if(num > 0)
    return 1;
  else if(num < 0)
    return -1;
  else
    return 0;
}
void Unicycle::satv(double * v){this->saturate(v,this->vmax,this->vmin);}
void Unicycle::satw(double * w){this->saturate(w,this->omegamax,this->omegamin);};
void Unicycle::saturate(double * val,double upper,double lower){
  //Only saturate if upper > lower and at least one of upper,lower is > -1
  if(upper < lower || upper < 0 || lower < 0)
    return;

  if(fabs(*val) > upper && upper > 0)
    *val = upper*this->sign(*val);
  else if(fabs(*val) < lower && lower > 0)
    *val = lower*sign(*val);

}

//Force theta into the +/- pi range
void Unicycle::normalizeTheta(double * theta){
  if(fabs(*theta) <= M_PI)
    return;

  if(*theta > 0){
    while(*theta > M_PI)
      *theta -= M_PI;
    return;
  }

  if(*theta < 0){
    while(*theta < -M_PI)
      *theta += M_PI;
    return;
  }
} 

void Unicycle::step(double * wsState,double * wsGrad,double * wsNewState){

  this->tmpSin = sin(this->currTheta);
  this->tmpCos = cos(this->currTheta);

  this->v = sqrt(pow(wsGrad[0],2) + pow(wsGrad[1],2));
  
  //Saturate v
  this->satv(&this->v);

  this->omega = (atan2(wsGrad[1],wsGrad[0]) - this->currTheta)/this->dt;

  //Saturate omega
  this->satw(&this->omega);
  
  //Integrate
  wsNewState[0] = wsState[0] + this->v*this->tmpCos*this->dt;
  wsNewState[1] = wsState[1] + this->v*this->tmpSin*this->dt;
  this->currTheta += this->omega*dt;

}

void Unicycle::saveState(){
  this->startTheta = this->currTheta;
}
