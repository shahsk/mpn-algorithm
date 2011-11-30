#ifndef __INTEGRATOR_H
#define __INTEGRATOR_H

#include "datatypes.h"

//Point dynamics and euler integration is given by default
//Omit an integrator class in cfg files to get this integrator
class Integrator{
 protected:
  int wsDim;
  mpn_float dt;
  unsigned int i;
 public:
  //For now, workspace dim is always 2
  Integrator(mpn_float stepTime,unsigned int workspaceDim);

  mpn_float getDt();

  //Integrate 1 step from current state
  virtual void step(mpn_float * wsState,mpn_float * wsGrad,
		    mpn_float * wsNewState);

  //Reset the internal state of this integrator
  virtual void reset();

  //Save the current state of the integrator as the start state
  virtual void saveState();

  virtual Integrator * copy();

};

#endif
