/*
 * Environment.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#include "datatypes.h"
#include "Environment.h"
#include "gamma.h"
#include <libconfig.h++>
#include <math.h>
#include <vector>
#include <string.h>

#include <iostream>

//return just beta value of the workspace
mpn_float Environment::calculateBeta0(mpn_float * q){
  envBeta = this->radpow2 - gamma(q,goal,this->dim);
  return envBeta;
}

//return beta of the whole workspace, including obstacles
//also refreshes internal beta values
//Defined as (r_0^2 - ||q-q_0||^2 ) * (product of all obstacle beta values)
mpn_float Environment::calculateBeta(mpn_float * q){
	mpn_float beta = calculateBeta0(q);
	if(obstacleBetaValues.size() != obstacles.size())
		obstacleBetaValues.resize(obstacles.size());
	for(unsigned int i(0); i<obstacles.size(); i++){
		obstacleBetaValues[i] = obstacles[i]->calculateBeta(q);
		beta *= obstacleBetaValues[i];
	}
	return beta;
}

Environment::Environment(mpn_float * destination,mpn_float k_in,mpn_float rad,
			 unsigned int dim){
  this->k = k_in;
  this->radius = rad;
  this->radpow2 = pow(rad,2);
  this->dim = dim;
  this->goal = new mpn_float[this->dim];
  for(unsigned int i(0); i<this->dim; i++){
    this->goal[i] = destination[i];
  }

}

Environment::Environment(libconfig::Setting & group,unsigned int dim){
  this->dim = dim;
  this->k = group["potential_parameter"];
  this->radius = group["radius"];
  this->radpow2 = pow(this->radius,2);
  this->goal = new mpn_float[this->dim];
  for(unsigned int i(0); i<this->dim; i++)
    this->goal[i] = group["destination"][i];
  
  if(group.exists("obstacles")){
    mpn_float tmpPos[this->dim],tmpR;
    for(unsigned int i(0); i<group["obstacles"].getLength(); i++){
      for(unsigned int j(0); j<this->dim; j++){
	tmpPos[j] = group["obstacles"][i]["position"][j];
      }
      tmpR = group["obstacles"][i]["radius"];
      this->obstacles.push_back(new Obstacle(tmpPos,tmpR));
    }
  }

}

Environment::~Environment() {
	// TODO Auto-generated destructor stub
}

//Calculate the value of the potential field at a given position q
//Defined as gamma(q,q_d) / ( (gamma(q,q_d)^k + beta)^1/k )
mpn_float Environment::potentialField(mpn_float * q){
  mpn_float gq = gamma(q,goal,this->dim);
  return gq / pow( pow(gq,k) + calculateBeta(q), 1/k);
}

//puts the negated gradient at q in answer
void Environment::negatedGradient(mpn_float * q,mpn_float * answer){
	mpn_float beta = calculateBeta(q); //calculate total beta, refresh values

	//Only happens when inside an obstacle
	if(beta < 0){
	  answer[0] = 0;
	  answer[1] = 0;
	  return;
	}

	mpn_float gam = gamma(q,goal,this->dim);
	mpn_float gamPowK = pow(gam,k);
	mpn_float dgam,tmp,beta0_partialTerm,betaPartial;

	for(unsigned int d(0); d<this->dim; d++){

		beta0_partialTerm = -2*(q[d]-goal[d]);//compute the first term separately, initialize with the partial of beta_0
		for(unsigned int i(0); i<obstacles.size(); i++){
			beta0_partialTerm *= obstacleBetaValues[i];
		}

		betaPartial = beta0_partialTerm;//initialize with the first term
		for(unsigned int i(0); i<obstacles.size(); i++){
			tmp = envBeta;
			for(unsigned int j(0); j<obstacles.size(); j++){
				if(j == i)//compute the partial
					tmp *= obstacles[i]->calculateDbeta(q,d);
				else//multiply betas from the rest of the obstacles
					tmp *= obstacleBetaValues[j];
			}
			betaPartial += tmp;
		}

		dgam = 2*(q[d]-goal[d]);
		answer[d] = -((dgam*pow(gamPowK+beta,1/k) - (1/k)*gam*pow(gamPowK+beta,1/k - 1)*(k*pow(gam,k-1)*dgam + betaPartial))
			/ pow(gamPowK+beta,2/k));
	}
}

DipolarEnvironment::DipolarEnvironment(mpn_float * destination,mpn_float k_in,
				       mpn_float rad,mpn_float ep,mpn_float goalOri):
  Environment(destination,k_in,rad,2){

  this->epsilon = ep;
  this->goalOrientation = goalOri;
  this->sinGoalOri = sin(goalOri);
  this->cosGoalOri = cos(goalOri);
}

DipolarEnvironment::DipolarEnvironment(libconfig::Setting & group):
  Environment(group,2){
  
  this->epsilon = group["epsilon"];
  this->goalOrientation = group["goal_orientation"];
  this->sinGoalOri = sin(this->goalOrientation);
  this->cosGoalOri = cos(this->goalOrientation);

}

void DipolarEnvironment::negatedGradient(mpn_float * q,mpn_float * answer){
	mpn_float beta = calculateBeta(q); //calculate total beta, refresh values

	//Only happens when inside an obstacle
	if(beta < 0){
	  answer[0] = 0;
	  answer[1] = 0;
	  return;
	}

	mpn_float gam = gamma(q,goal,this->dim);
	mpn_float gamPowK = pow(gam,k);
	mpn_float rotVector[2] = {cosGoalOri,sinGoalOri};
	mpn_float hval = pow((q[0] - goal[0])*cosGoalOri + 
			  (q[1] - goal[1])*cosGoalOri,2) + epsilon;
	mpn_float hTmp = 2*((q[0]-goal[0])*cosGoalOri + 
			    (q[1]-goal[1])*sinGoalOri);
	mpn_float dgam,tmp,beta0_partialTerm,betaPartial,hPartial;

	for(int d(0); d<this->dim; d++){

		beta0_partialTerm = -2*(q[d]-goal[d]);//compute the first term separately, initialize with the partial of beta_0
		for(unsigned int i(0); i<obstacles.size(); i++){
			beta0_partialTerm *= obstacleBetaValues[i];
		}

		betaPartial = beta0_partialTerm;//initialize with the first term
		for(unsigned int i(0); i<obstacles.size(); i++){
			tmp = envBeta;
			for(unsigned int j(0); j<obstacles.size(); j++){
				if(j == i)//compute the partial
					tmp *= obstacles[i]->calculateDbeta(q,d);
				else//multiply betas from the rest of the obstacles
					tmp *= obstacleBetaValues[j];
			}
			betaPartial += tmp;
		}

		dgam = 2*(q[d]-goal[d]);
		hPartial = hTmp*rotVector[d];
		answer[d] = -((dgam*pow(gamPowK+beta*hval,1/k) - 
			       (1/k)*gam*pow(gamPowK+beta*hval,1/k - 1)*
			       (k*pow(gam,k-1)*dgam + 
			       (betaPartial*hval + beta*hPartial)))
			       / pow(gamPowK+beta*hval,2/k));
	}

}

mpn_float DipolarEnvironment::potentialField(mpn_float * q){
  mpn_float gq = gamma(q,goal,this->dim);
  mpn_float eta = pow((q[0] - goal[0])*cosGoalOri + 
		   (q[1] - goal[1])*cosGoalOri,2);
  return gq / pow( pow(gq,k) + ((epsilon + eta)*calculateBeta(q)), 1/k);
}
