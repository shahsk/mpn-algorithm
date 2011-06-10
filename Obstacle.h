/*
 * Obstacle.h
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "Config.h"

class Obstacle {
private:
	double pos[DIM];//x,y(,z) position
	double radius;//size of the obstacle
public:
	Obstacle(double * position,double r);
	virtual ~Obstacle();

	double calculateBeta(double* q);//calculate the beta value of this obstacle at a given point
	double calculateDbeta(double* q,int dim);//calculate the given partial of beta at a given point
};

double gamma(double*,double*);
double gamma(double*);

#endif /* OBSTACLE_H_ */
