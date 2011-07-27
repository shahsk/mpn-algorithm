#ifndef __UNICYCLE_H
#define __UNICYCLE_H

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
  double vmax,vmin,omegamax,omegamin;

  double dirVector[2];

  //Temporaries
  double vdot,v,omega,tmpSin,tmpCos;

  void satv(double * v);
  void satw(double * w);
  void normalizeTheta(double * theta); 

 public:
  double startTheta;
  double currTheta;
  Unicycle(double stepTime,double theta, double vmax = -1,double omegamax = -1,
	   double vmin = -1,double omegamin = -1);
  //Dim is ignored, Unicycle is 2D only
  Unicycle(libconfig::Setting& group,double stepTime,unsigned int dim);

  //Euler integration
  void step(double * wsState,double * wsGrad,double * wsNewState);

  //Reset the internal state of this integrator
  void reset();

  //Save the current internal state as the initial state
  void saveState();

  Integrator * copy();
};

#endif
