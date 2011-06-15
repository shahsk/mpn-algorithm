/*
 * MPN2D.h
 *
 *  Created on: Jun 9, 2011
 *      Author: nlacock
 */

#ifndef MPN2D_H_
#define MPN2D_H_

#include "Environment.h"

struct MPNParams{
	unsigned int nLegendrePolys;
	double * controlParameters;
	double confidence,level,predictionHorizon,controlHorizon,currentTime;
	double costWeights[3];
};

//Allocates a given number of double[2] points
double ** allocatePoints(int npoints);

//Deallocates a given number of double[2] points, meant to be used in conjunction with allocatePoints
void cleanupPoints(double ** pointArray,int npoints);

//Puts the nominal controlPath and the path in the given variables
void nominalPath(Environment & e,double** controlPath, double ** path,double * start, double dt, int steps);

//Puts a sample controlPath and path into the given variables, given a set of parameters
void samplePath(Environment & e, MPNParams & params,double** controlPath, double ** path,double * start, double dt, int steps);

//Returns the angle of perturbation generated by MPNParams for a given time
double anglePerturb(MPNParams & params, double * gradient,double polytime);

//Returns the value of the potential field at the endpoint of a given trajectory
double terminalCost(Environment & e,double ** path,int size);

//Dummy cost function that always returns 0
double noExtraCost(Environment & e,double * state);

//Returns the cost of a given path and control trajectory, applies the given cost function
double incrementalCost(Environment & e, MPNParams & params,double ** path, double ** controlPath, double dt, int size, double(*extraCost)(Environment &,double *) = noExtraCost);

/* Runs the bulk of the algorithm.
 * 1. compute the nominal path
 * 2. generate ln(1/(1-confidence)) / ln(1/(1-level)) paths such that
 *    cost(path at control horizon time) < cost(current time)
 *    TODO: I think there are some more restrictions on acceptable paths
 * 3. find the combination of control parameters and path that gives the lowest cost
 *
 * The best control parameters get put in params.controlParameters
 * bestPath will point to a newly allocated best path
 * bestControl will be the control inputs along the path
 * steps will contain the length of the path
 * controlHorizonIndex will be the index of the control horizon along the path
 * */
void generateBestPath(Environment & e, MPNParams & params, double ** &bestPath, double **& bestControl, int & steps, int & controlHorizonIndex,  double * start, double dt);

#endif /* MPN2D_H_ */
