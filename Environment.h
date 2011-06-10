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
	double k;//tuning parameter for potential field
	double radius;//size of the workspace

	std::vector<double> obstacleBetaValues;
	double envBeta;

public:

	double goal[DIM];
	std::vector<Obstacle> obstacles;

	Environment(double * destination,double k_in,double rad);
	double potentialField(double * q); //returns the value of the potential field at a given point
	void negatedGradient(double * q,double * answer); //puts the negated gradient at q in answer
	double calculateBeta(double * q);//return beta of the whole workspace, including obstacles. also refreshes internal beta values
	double calculateBeta0(double * q); //return just beta value of the workspace

	virtual ~Environment();
};

#endif /* ENVIRONMENT_H_ */
