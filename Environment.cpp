/*
 * Environment.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#include "Environment.h"
#include <math.h>
#include <vector>

//return just beta value of the workspace
double Environment::calculateBeta0(double * q){
	envBeta = pow(radius,2)- gamma(q,goal);
	return envBeta;
}

//return beta of the whole workspace, including obstacles
//also refreshes internal beta values
//Defined as (r_0^2 - ||q-q_0||^2 ) * (product of all obstacle beta values)
double Environment::calculateBeta(double * q){
	double beta = calculateBeta0(q);
	if(obstacleBetaValues.size() != obstacles.size())
		obstacleBetaValues.resize(obstacles.size());
	for(unsigned int i(0); i<obstacles.size(); i++){
		obstacleBetaValues[i] = obstacles[i].calculateBeta(q);
		beta *= obstacleBetaValues[i];
	}
	return beta;
}

Environment::Environment(double * destination,double k_in,double rad){
  k = k_in;
  radius = rad;
  for(int i(0); i<DIM; i++)
    goal[i] = destination[i];

  vmax = .5;
  vmin = 0;
  omegamax = 3;
  omegamin = 0;
  
}

Environment::~Environment() {
	// TODO Auto-generated destructor stub
}

//Calculate the value of the potential field at a given position q
//Defined as gamma(q,q_d) / ( (gamma(q,q_d)^k + beta)^1/k )
double Environment::potentialField(double * q){
	double gq = gamma(q,goal);
	return gq / pow( pow(gq,k) + calculateBeta(q), 1/k);
}

//puts the negated gradient at q in answer
void Environment::negatedGradient(double * q,double * answer){
	double beta = calculateBeta(q); //calculate total beta, refresh values
	if(beta < 0)
		return;

	double gam = gamma(q,goal);
	double gamPowK = pow(gam,k);
	double dgam,tmp,beta0_partialTerm,betaPartial;

	for(int dim(0); dim<DIM; dim++){

		beta0_partialTerm = -2*(q[dim]-goal[dim]);//compute the first term separately, initialize with the partial of beta_0
		for(unsigned int i(0); i<obstacles.size(); i++){
			beta0_partialTerm *= obstacleBetaValues[i];
		}

		betaPartial = beta0_partialTerm;//initialize with the first term
		for(unsigned int i(0); i<obstacles.size(); i++){
			tmp = envBeta;
			for(unsigned int j(0); j<obstacles.size(); j++){
				if(j == i)//compute the partial
					tmp *= obstacles[i].calculateDbeta(q,dim);
				else//multiply betas from the rest of the obstacles
					tmp *= obstacleBetaValues[j];
			}
			betaPartial += tmp;
		}

		dgam = 2*(q[dim]-goal[dim]);
		answer[dim] = -((dgam*pow(gamPowK+beta,1/k) - (1/k)*gam*pow(gamPowK+beta,1/k - 1)*(k*pow(gam,k-1)*dgam + betaPartial))
			/ pow(gamPowK+beta,2/k));
	}
}

void Environment::integrator(double * q,double * negGrad,double & currentOri, double dt, double * ans){

  tmpSin = sin(currentOri);
  tmpCos = cos(currentOri);

  vdot = negGrad[0]*tmpCos + negGrad[1]*tmpSin;
  v = sqrt(pow(negGrad[0],2) + pow(negGrad[1],2)) + vdot*dt;

  if(!this->vset){
    this->prevV = sqrt(gamma(negGrad));
    this->vset = true;
  }
  v = this->prevV + vdot*dt;

  
  if(v > 0){
    if(v > vmax)
      v = vmax;
    if(v < vmin)
      v = vmin;
  }
  if(v < 0){
    if(v < -vmax)
      v = -vmax;
    if(v > -vmin)
      v = -vmin;
  }
  prevV = v;
  omega = (negGrad[1]*tmpCos - negGrad[0]*tmpSin)/v;
  
  if(omega > 0){
    if(omega > omegamax)
      omega = omegamax;
    if(omega < omegamin)
      omega = omegamin;
  }
  if(omega < 0){
    if(omega < -omegamax)
      omega = -omegamax;
    if(omega > -omegamin)
      omega = -omegamin;
  }
  
  ans[0] = q[0] + v*tmpCos*dt;
  ans[1] = q[1] + v*tmpSin*dt;
  currentOri = currentOri + omega*dt;


  //ans[0] = q[0] + dt*negGrad[0];
  //ans[1] = q[1] + dt*negGrad[1];
} 


DipolarEnvironment::DipolarEnvironment(double * destination,double k_in,
				       double rad,double ep,double goalOri):
  Environment(destination,k_in,rad){

  epsilon = ep;
  goalOrientation = goalOri;
  sinGoalOri = sin(goalOri);
  cosGoalOri = cos(goalOri);
  //vset = false;
}

void DipolarEnvironment::negatedGradient(double * q,double * answer){
	double beta = calculateBeta(q); //calculate total beta, refresh values
	if(beta < 0)
		return;

	double gam = gamma(q,goal);
	double gamPowK = pow(gam,k);
	double rotVector[2] = {cosGoalOri,sinGoalOri};
	double hval = pow((q[0] - goal[0])*cosGoalOri + 
			  (q[1] - goal[1])*cosGoalOri,2) + epsilon;
	double hTmp = 2*((q[0]-goal[0])*cosGoalOri + 
			    (q[1]-goal[1])*sinGoalOri);
	double dgam,tmp,beta0_partialTerm,betaPartial,hPartial;

	for(int dim(0); dim<DIM; dim++){

		beta0_partialTerm = -2*(q[dim]-goal[dim]);//compute the first term separately, initialize with the partial of beta_0
		for(unsigned int i(0); i<obstacles.size(); i++){
			beta0_partialTerm *= obstacleBetaValues[i];
		}

		betaPartial = beta0_partialTerm;//initialize with the first term
		for(unsigned int i(0); i<obstacles.size(); i++){
			tmp = envBeta;
			for(unsigned int j(0); j<obstacles.size(); j++){
				if(j == i)//compute the partial
					tmp *= obstacles[i].calculateDbeta(q,dim);
				else//multiply betas from the rest of the obstacles
					tmp *= obstacleBetaValues[j];
			}
			betaPartial += tmp;
		}

		dgam = 2*(q[dim]-goal[dim]);
		hPartial = hTmp*rotVector[dim];
		answer[dim] = -((dgam*pow(gamPowK+beta*hval,1/k) - 
			       (1/k)*gam*pow(gamPowK+beta*hval,1/k - 1)*
			       (k*pow(gam,k-1)*dgam + 
			       (betaPartial*hval + beta*hPartial)))
			       / pow(gamPowK+beta*hval,2/k));
	}

}

double DipolarEnvironment::potentialField(double * q){
  double gq = gamma(q,goal);
  double eta = pow((q[0] - goal[0])*cosGoalOri + 
		   (q[1] - goal[1])*cosGoalOri,2);
  return gq / pow( pow(gq,k) + ((epsilon + eta)*calculateBeta(q)), 1/k);
}

void DipolarEnvironment::integrator(double * q,double * negGrad,double & currentOri, double dt, double * ans){

  //double vdot,v,omega,tmpSin,tmpCos;
  //double vmax = .5,vmin = 0,omegamax = 3,omegamin = 0;
  tmpSin = sin(currentOri);
  tmpCos = cos(currentOri);

  vdot = negGrad[0]*tmpCos + negGrad[1]*tmpSin;
  v = sqrt(pow(negGrad[0],2) + pow(negGrad[1],2)) + vdot*dt;

  if(!this->vset){
    this->prevV = sqrt(gamma(negGrad));
    this->vset = true;
  }
  v = this->prevV + vdot*dt;

  
  if(v > 0){
    if(v > vmax)
      v = vmax;
    if(v < vmin)
      v = vmin;
  }
  if(v < 0){
    if(v < -vmax)
      v = -vmax;
    if(v > -vmin)
      v = -vmin;
  }
  prevV = v;
  omega = (negGrad[1]*tmpCos - negGrad[0]*tmpSin)/v;
  
  if(omega > 0){
    if(omega > omegamax)
      omega = omegamax;
    if(omega < omegamin)
      omega = omegamin;
  }
  if(omega < 0){
    if(omega < -omegamax)
      omega = -omegamax;
    if(omega > -omegamin)
      omega = -omegamin;
  }
  
  ans[0] = q[0] + v*tmpCos*dt;
  ans[1] = q[1] + v*tmpSin*dt;
  currentOri = currentOri + omega*dt;
  

  /*
  double vel = negGrad[0]*cos(currentOri) + negGrad[1]*sin(currentOri);
  ans[0] = q[0] + vel*cos(currentOri)*dt;
  ans[1] = q[1] + vel*sin(currentOri)*dt;
  double omega = atan2(negGrad[1],negGrad[0]) - currentOri;
  currentOri += omega*dt;
  */
}
