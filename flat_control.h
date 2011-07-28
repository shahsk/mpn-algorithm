#ifndef __FLAT_CONTROL_H
#define __FLAT_CONTROL_H

#include "alglib/interpolation.h"
//#include "traj_control.h"
#include <libplayerc++/playerc++.h>
#include <time.h>

//Does not check angle for goal condition
class flat_control {//: public traj_control{
 protected:
  PlayerCc::PlayerClient * sourceClient,* sinkClient;
  PlayerCc::Position2dProxy * sourcePos,* sinkPos;
  double endGoal[2],tol,vmax,wmax,vmin,wmin;

  double goal[2],pose[3],currVel[2],desiredV[2],desiredX[2],desiredA[2],u[2],v,
    omega,acc,dt,steps;
  
  timeval start,now;
  double startTime,currTime,prevTime;

  //alglib::barycentricinterpolant * xCurrent,* yCurrent, * xNext, * yNext;
  alglib::spline1dinterpolant * xCurrent,* yCurrent, * xNext, * yNext;
  double currDuration,nextDuration;

  bool first;
 public:
  
  //If position reading is done on a different interface from position commands
  flat_control(PlayerCc::PlayerClient * sourceClient,
	       PlayerCc::Position2dProxy * sourcePos,
	       double * endGoal,double tol,
	       PlayerCc::PlayerClient * sinkClient = NULL,
	       PlayerCc::Position2dProxy * sinkPos = NULL,
	       double vmax = -1,double omegamax = -1,double vmin = -1,
	       double omegamin = -1);

  void preProcess(double time,double ** path, double ** pathDeriv, 
			  int steps);

  void operator()();
};

#endif
