#ifndef __TRAJ_CONTROL_H
#define __TRAJ_CONTROL_H

class traj_control{

 public:  

  //This operation should be threadsafe, it is meant to be called
  //while the driving loop is running
  virtual void preProcess(double time,double ** path, double ** pathDeriv, 
			  int steps) = 0;

  //This should be thread-safe, and actually drive the robot
  virtual void operator()(){}

};

#endif
