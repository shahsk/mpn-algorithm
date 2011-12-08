#include "datatypes.h"
#include "Integrator.h"

#ifdef NEON
#include <arm_neon.h>
#endif

Integrator::Integrator(mpn_float stepTime,unsigned int workspaceDim){
    this->wsDim = workspaceDim;
    this->dt = stepTime;
}

mpn_float Integrator::getDt(){return this->dt;}

void Integrator::step(mpn_float * wsState,mpn_float * wsGrad,mpn_float * wsNewState){
#ifndef DIM2
  for(i = 0; i < this->wsDim; i++){
    wsNewState[i] = wsState[i] + wsGrad[i]*this->dt;
  }
#else

#if (defined SINGLE_PRECISION && defined NEON)
  float32x2_t x = vld1_f32(wsState);
  float32x2_t dx = vld1_f32(wsGrad);
  float32x2_t dt = vdup_n_f32(this->dt);

  x = vmla_f32(x,dx,dt);
  vst1_f32(wsNewState,x);

#else
  wsNewState[0] = wsState[0] + wsGrad[0]*this->dt;
  wsNewState[1] = wsState[1] + wsGrad[1]*this->dt;

#endif

#endif

}

void Integrator::reset(){}

void Integrator::saveState(){}

Integrator* Integrator::copy(){
  return new Integrator(this->dt,this->wsDim);
}
