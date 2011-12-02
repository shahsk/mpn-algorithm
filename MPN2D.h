/*
 * MPN2D.h
 *
 *  Created on: Jun 9, 2011
 *      Author: nlacock
 */

#ifndef MPN2D_H_
#define MPN2D_H_

#include "datatypes.h"
#include "Environment.h"
#include "MPNParams.h"
#include "Integrator.h"
#include "gamma.h"

//This is a 2D implementation only
#define DIM 2

//Allocates a given number of mpn_float[2] points
mpn_float * allocatePoints(int npoints);

//Deallocates a given number of mpn_float[2] points, meant to be used in conjunction with allocatePoints
void cleanupPoints(mpn_float * pointArray,int npoints);
/*
//Returns the value of the potential field at the endpoint of a given trajectory
mpn_float inline terminalCost(Environment * e,mpn_float * path,int size);
*/
//Dummy cost function that always returns 0
mpn_float noExtraCost(Environment * e,mpn_float * state);

//Returns the cost of a given path and control trajectory, applies the given cost function
mpn_float incrementalCost(Environment * e, const MPNParams * params,mpn_float * path, mpn_float * controlPath, mpn_float dt, int size, mpn_float(*extraCost)(Environment *,mpn_float *) = noExtraCost);

//Puts the nominal controlPath and the path in the given variables
int nominalPath(Environment * e,Integrator * intgr,mpn_float* controlPath, mpn_float * path,mpn_float * start,int steps,Integrator *& atCHI,const MPNParams * params = NULL);

//Puts a sample controlPath and path into the given variables, given a set of parameters
int samplePath(Environment * e, Integrator * intgr,const MPNParams * params,mpn_float* controlPath, mpn_float * path,mpn_float * start,int steps,Integrator *& atCHI);

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
bool generateBestPath(Environment * e, MPNParams * params, Integrator *& intgr, mpn_float * &bestPath, mpn_float *& bestControl, int & steps, int & controlHorizonIndex,  mpn_float * start);


#endif /* MPN2D_H_ */
