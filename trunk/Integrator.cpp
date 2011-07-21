#include "Integrator.h"
#include <libconfig.h++>

Integrator::Integrator(double stepTime,unsigned int workspaceDim){
    this->wsDim = workspaceDim;
    this->dt = stepTime;
}

double Integrator::getDt(){return this->dt;}

void Integrator::step(double * wsState,double * wsGrad,double * wsNewState){
  for(i = 0; i < this->wsDim; i++){
    wsNewState[i] = wsState[i] + wsGrad[i]*this->dt;
  }
}

void Integrator::reset(){}

void Integrator::saveState(){}
