/*
 * Obstacle.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#include "Obstacle.h"
#include "Config.h"
#include <math.h>

//calculate the beta value of this obstacle
//defined as ||q-q_i||^2 - r_i^2 for an obstacle i with radius r and position q
//TODO: In the paper and in MPN_gui there are different beta functions for obstacles, the paper uses
//||q-q_i|| - r_i^2
double Obstacle::calculateBeta(double* q){
	return gamma(q,pos) - pow(radius,2);
}

//calculate the given partial of beta at a given point
//e.g. dim = 0 for dbeta/dx, dim = 1 for dbeta/dy etc.
double Obstacle::calculateDbeta(double* q,int dim){
	return 2*(q[dim]-pos[dim]);
}

Obstacle::Obstacle(double * position,double r){
	for(int i(0); i<DIM; i++)
		pos[i] = position[i];
	radius = r;
}

//Returns the n-dimensional distance SQUARED between 2 points
double gamma(double * p1, double * p2){
	double mag = 0;
	for(int i(0); i<DIM; i++)
		mag += pow(p1[i] - p2[i],2);
	return mag;
}

//Returns the n-dimensional distance SQUARED to the origin
double gamma(double* p){
	double mag = 0;
	for(int i(0); i<DIM; i++)
		mag += pow(p[i],2);
	return mag;
}

Obstacle::~Obstacle() {
	// TODO Auto-generated destructor stub
}
