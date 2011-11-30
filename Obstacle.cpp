/*
 * Obstacle.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#include "datatypes.h"
#include "Obstacle.h"
#include <math.h>
#include "gamma.h"

//calculate the beta value of this obstacle
//defined as ||q-q_i||^2 - r_i^2 for an obstacle i with radius r and position q
//TODO: In the paper and in MPN_gui there are different beta functions for obstacles, the paper uses
//||q-q_i|| - r_i^2
mpn_float Obstacle::calculateBeta(mpn_float* q){
  return gamma(q,pos,this->dim) - pow(radius,2);
}

//calculate the given partial of beta at a given point
//e.g. dim = 0 for dbeta/dx, dim = 1 for dbeta/dy etc.
mpn_float Obstacle::calculateDbeta(mpn_float* q,unsigned int d){
	return 2*(q[d]-pos[d]);
}

Obstacle::Obstacle(mpn_float * position,mpn_float r,unsigned int dim){
  this->dim = dim;
  this->pos = new mpn_float[this->dim];
  for(unsigned int i(0); i<this->dim; i++)
    pos[i] = position[i];
  this->radius = r;
}
 
Obstacle::~Obstacle() {
  delete [] this->pos;
}
