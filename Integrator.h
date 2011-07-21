#ifndef __INTEGRATOR_H
#define __INTEGRATOR_H

//Point dynamics and euler integration is given by default
//Omit an integrator class in cfg files to get this integrator
class Integrator{
 protected:
  int wsDim;
  double dt;
  unsigned int i;
 public:
  //For now, workspace dim is always 2
  Integrator(double stepTime,unsigned int workspaceDim);

  double getDt();

  //Integrate 1 step from current state
  virtual void step(double * wsState,double * wsGrad,double * wsNewState);

  //Reset the internal state of this integrator
  virtual void reset();

  //Save the current state of the integrator as the start state
  virtual void saveState();

};

#endif
