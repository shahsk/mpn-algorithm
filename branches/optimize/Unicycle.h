#ifndef __UNICYCLE_H
#define __UNICYCLE_H

#include "datatypes.h"
#include "Integrator.h"
#include <libconfig.h++>

#define UNICYCLE_INTEGRATOR "unicycle_integrator"

/*
  Unicycle robot dynamics:
  dx/dt = V*cos(theta)
  dy/dt = V*sin(theta)
  dtheta/dt = W

  Uses V = norm(gradient), W = (atan(gradient) - currentAngle)
  
  This model also allows for saturation of V and W to do realistic planning

  Omitting optional fields results in absence of constraint. Constraint fields
  must be positive floats, otherwise they will be ignored.

  Config grammar example:

  unicycle_integrator:{
    #Required fields
    start_orientation = 3.14;
    
    #Optional fields
    max_velocity = 1.0;
    min_velocity = 0.0;
    max_angular_velocity = 1.0;
    min_angular_velocity = 0.0;
  
  };

*/
class Unicycle: public Integrator{
 protected:


  //Constraints
  mpn_float vmax,vmin,omegamax,omegamin;

  mpn_float dirVector[2];

  //Temporaries
  mpn_float vdot,v,omega,tmpSin,tmpCos;

  void satv(mpn_float * v);
  void satw(mpn_float * w);
  void normalizeTheta(mpn_float * theta); 

 public:
  mpn_float startTheta;
  mpn_float currTheta;
  Unicycle(mpn_float stepTime,mpn_float theta, mpn_float vmax = -1,mpn_float omegamax = -1,
	   mpn_float vmin = -1,mpn_float omegamin = -1);
  //Dim is ignored, Unicycle is 2D only
  Unicycle(libconfig::Setting& group,mpn_float stepTime,unsigned int dim);

  //Euler integration
  void step(mpn_float * wsState,mpn_float * wsGrad,mpn_float * wsNewState);

  //Reset the internal state of this integrator
  void reset();

  //Save the current internal state as the initial state
  void saveState();

  Integrator * copy();
};

#endif
