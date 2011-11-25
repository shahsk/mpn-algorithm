#include "Integrator.h"

Integrator::Integrator(double stepTime,unsigned int workspaceDim){
    this->wsDim = workspaceDim;
    this->dt = stepTime;
}

double Integrator::getDt(){return this->dt;}

void Integrator::step(double * wsState,double * wsGrad,double * wsNewState){
#ifndef DIM2
  for(i = 0; i < this->wsDim; i++){
    wsNewState[i] = wsState[i] + wsGrad[i]*this->dt;
  }
#else
  wsNewState[0] = wsState[0] + wsGrad[0]*this->dt;
  wsNewState[1] = wsState[1] + wsGrad[1]*this->dt;
#endif

}

void Integrator::reset(){}

void Integrator::saveState(){}

Integrator* Integrator::copy(){
  return new Integrator(this->dt,this->wsDim);
}
