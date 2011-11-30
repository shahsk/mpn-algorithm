#include "datatypes.h"
#include "Unicycle.h"
#include "saturate.h"
#include "math.h"
#include <iostream>

Unicycle::Unicycle(mpn_float stepTime,mpn_float theta,mpn_float vmax,mpn_float omegamax,
		   mpn_float vmin,mpn_float omegamin): Integrator(stepTime,2){
  this->startTheta = theta;
  this->vmax = vmax;
  this->omegamax = omegamax;
  this->vmin = vmin;
  this->omegamin = omegamin;

  this->currTheta = theta;
  this->dirVector[0] = cos(this->currTheta);
  this->dirVector[1] = sin(this->currTheta);
  
}

Unicycle::Unicycle(libconfig::Setting& group,mpn_float stepTime,unsigned int dim):
  Integrator(stepTime,2){
  double tmp;
  tmp = group["start_orientation"];
  this->startTheta = tmp;
  this->currTheta = this->startTheta;
  
  this->dirVector[0] = cos(this->currTheta);
  this->dirVector[1] = sin(this->currTheta);

  this->vmax = -1;
  this->vmin = -1;
  this->omegamax = -1;
  this->omegamin = -1;

  group.lookupValue("max_velocity",tmp);
  this->vmax = tmp;
  group.lookupValue("min_velocity",tmp);
  this->vmin = tmp;
  group.lookupValue("max_angular_velocity",tmp);
  this->omegamax = tmp;
  group.lookupValue("min_angular_velocity",tmp);
  this->omegamin = tmp;

}

void Unicycle::reset(){
  this->currTheta = this->startTheta;
}

void Unicycle::satv(mpn_float * v){saturate(v,this->vmax,this->vmin);}
void Unicycle::satw(mpn_float * w){saturate(w,this->omegamax,this->omegamin);};

//Force theta into the +/- pi range
void Unicycle::normalizeTheta(mpn_float * theta){
  while(*theta > 2*M_PI)
    *theta = *theta-2*M_PI;
  while(*theta < 0)
    *theta = *theta+2*M_PI;
} 

void Unicycle::step(mpn_float * wsState,mpn_float * wsGrad,mpn_float * wsNewState){

  this->normalizeTheta(&this->currTheta);
  mpn_float desired = atan2(wsGrad[1],wsGrad[0]);
  mpn_float difference = desired - this->currTheta;
  this->normalizeTheta(&desired);


  if(difference > M_PI)
    difference -= 2*M_PI;
  else if(difference < -M_PI)
    difference += 2*M_PI;

  this->omega = difference/this->dt;
  this->satw(&this->omega);

  this->v = sqrt(pow(wsGrad[0],2) + pow(wsGrad[1],2));
  this->satv(&this->v);

  this->currTheta += this->omega*this->dt;  
  wsGrad[0] = this->v*cos(this->currTheta);
  wsGrad[1] = this->v*sin(this->currTheta);
  wsNewState[0] = wsState[0] + wsGrad[0]*this->dt;
  wsNewState[1] = wsState[1] + wsGrad[1]*this->dt;

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
