#include "Unicycle.h"
#include "math.h"
#include <iostream>

Unicycle::Unicycle(double stepTime,double theta,double vmax,double omegamax,
		   double vmin,double omegamin): Integrator(stepTime,2){
  this->startTheta = theta;
  this->vmax = vmax;
  this->omegamax = omegamax;
  this->vmin = vmin;
  this->omegamin = omegamin;

  this->currTheta = theta;
  this->dirVector[0] = cos(this->currTheta);
  this->dirVector[1] = sin(this->currTheta);
  
}

Unicycle::Unicycle(libconfig::Setting& group,double stepTime,unsigned int dim):
  Integrator(stepTime,2){
  
  this->startTheta = group["start_orientation"];
  this->currTheta = this->startTheta;
  
  this->dirVector[0] = cos(this->currTheta);
  this->dirVector[1] = sin(this->currTheta);

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
  if(fabs(*val) > upper && upper > 0){
    *val = this->sign(*val)*upper;
  }
  else if(*val < lower && lower > 0){
    *val = this->sign(*val)*lower;
  }

}

//Force theta into the +/- pi range
void Unicycle::normalizeTheta(double * theta){
  if(*theta > 2*M_PI)
    *theta -= 2*M_PI;
  if(*theta < 0)
    *theta += 2*M_PI;
} 

void Unicycle::step(double * wsState,double * wsGrad,double * wsNewState){

  this->normalizeTheta(&this->currTheta);
  double desired = atan2(wsGrad[1],wsGrad[0]);
  this->normalizeTheta(&desired);  

  double difference = desired - this->currTheta;
  if(difference > M_PI)
    difference -= 2*M_PI;
  else if(difference < -M_PI)
    difference += 2*M_PI;

  this->omega = difference/this->dt;
  this->satw(&this->omega);

  this->v = sqrt(pow(wsGrad[0],2) + pow(wsGrad[1],2));
  this->satv(&this->v);

  this->currTheta += this->omega*this->dt;  
  wsNewState[0] = wsState[0] + this->v*cos(this->currTheta)*this->dt;
  wsNewState[1] = wsState[1] + this->v*sin(this->currTheta)*this->dt;
  
}

void Unicycle::saveState(){
  this->startTheta = this->currTheta;
}

Integrator * Unicycle::copy(){
  Unicycle * out = new Unicycle(this->dt,this->startTheta,this->vmax,
				this->omegamax,this->vmin,this->omegamin);
  out->currTheta = this->currTheta;
  return out;  
}
