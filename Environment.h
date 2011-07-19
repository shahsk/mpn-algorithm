/*
 * Environment.h
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "Obstacle.h"
#include <vector>

class Environment {//assumed to be centered at 0,0
 protected:
  double k;//tuning parameter for potential field
  double radius;//size of the workspace
  
  std::vector<double> obstacleBetaValues;
  double envBeta;


 public:
  double prevV;
  bool vset;
  
  //Temporaries
  double vdot,v,omega,tmpSin,tmpCos;
  double vmax,vmin,omegamax,omegamin;


  double goal[DIM];
  std::vector<Obstacle> obstacles;

  Environment(double * destination,double k_in,double rad);
  
  double calculateBeta(double * q);//return beta of the whole workspace, including obstacles. also refreshes internal beta values
  double calculateBeta0(double * q); //return just beta value of the workspace
  
  //puts the negated gradient at q in answer
  virtual void negatedGradient(double * q,double * answer);
  //returns the value of the potential field at a given point
  virtual double potentialField(double * q);
  //integrates 1 step. Simple euler integration of point dynamics by default
  virtual void integrator(double * q,double * negGrad,double & currentOri, double dt, double * ans); 
  
  virtual ~Environment();
};

class DipolarEnvironment: public Environment{
 private:
  double epsilon,goalOrientation,sinGoalOri,cosGoalOri;



 public:
  DipolarEnvironment(double * destination,double k_in,double rad,
		     double ep,double goalOri);

  void negatedGradient(double * q,double * answer);
  double potentialField(double * q);
  void integrator(double * q,double * negGrad,double & currentOri, double dt, double * ans); 

};


#endif /* ENVIRONMENT_H_ */
