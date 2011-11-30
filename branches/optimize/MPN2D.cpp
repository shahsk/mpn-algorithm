/*
 * MPN2D.cpp
 *
 *  Created on: Jun 9, 20111
 *      Author: nlacock
 */

#include "datatypes.h"
#include "MPN2D.h"
#include "Integrator.h"
#include "Unicycle.h"
#include "Environment.h"
#include "specialfunctions.h"
//#include "gamma.h"

#include <cmath>
#include <iostream>

#define MAX_TRIES 10000

mpn_float inline noExtraCost(Environment * e,mpn_float * state){return 0.;}
/*
//Allocates a given number of mpn_float[2] points
mpn_float ** allocatePoints(int npoints){
	mpn_float ** output = new mpn_float*[npoints];
	for(int i(0); i<npoints; i++)
		output[i] = new mpn_float[2];
	return output;
}
*/
mpn_float *allocatePoints(int npoints){
  //evens are the x's, odds are the y's,  i.e. x1, y1, x2, y2, ...
  return new mpn_float[npoints*2];
}
/*
//Deallocates a given number of mpn_float[2] points, meant to be used in conjunction with allocatePoints
void cleanupPoints(mpn_float ** pointArray,int npoints){
	for(int i(0); i<npoints; i++)
		delete [] pointArray[i];
	delete [] pointArray;
}
*/
void cleanupPoints(mpn_float *pointArray){
  delete [] pointArray;
}

/*
//Size is the index of the path to use, usually the number of steps
mpn_float inline terminalCost(Environment * e,mpn_float ** path,int size){
	return e->potentialField(path[size-1]);
}
*/

mpn_float inline terminalCost(Environment *e, mpn_float * path, int size)
{
  //Send a pointer to section of array with desired x/y
  return e->potentialField(&path[2*(size-1)]);
}
    /*

//Size should be the actual size of path and controlPath
//TODO: Terminal cost should have a different weight as well
mpn_float incrementalCost(Environment * e, MPNParams * params,mpn_float ** path, mpn_float ** controlPath, mpn_float dt, int size, mpn_float(*extraCost)(Environment *,mpn_float *)){
  
  //compute the value of the cost function at every point
  mpn_float costFunction[size];
  for(int i(0); i<size; i++){
    costFunction[i] = params->costWeights[0]*gamma(path[i],e->goal,DIM) +
      params->costWeights[1]*gamma(controlPath[i],DIM);
  }
  
  if(extraCost != noExtraCost){
    for(int i(0); i<size; i++){
      costFunction[i] += params->costWeights[2]*extraCost(e,path[i]);
    }
  }

  //do a cumulative integration of the cost function (wrt time) using the trapezoidal method
  mpn_float cost = costFunction[0] + costFunction[size-1];
  for(int i(1); i<size-1; i++){
    cost += 2*costFunction[i];
  }
  
  return (dt/2.)*cost;
}
    */

//Size should be the actual size of path and controlPath
//TODO: Terminal cost should have a different weight as well
mpn_float incrementalCost(Environment * e, MPNParams * params,mpn_float * path, mpn_float * controlPath, mpn_float dt, int size, mpn_float(*extraCost)(Environment *,mpn_float *)){
  
  //compute the value of the cost function at every point
  mpn_float costFunction[size];
  for(int i(0); i<size; i++){
    costFunction[i] = params->costWeights[0]*gamma(&path[2*i],e->goal,DIM) +
      params->costWeights[1]*gamma(&controlPath[2*i],DIM);
  }
  
  if(extraCost != noExtraCost){
    for(int i(0); i<size; i++){
      costFunction[i] += params->costWeights[2]*extraCost(e,&path[2*i]);
    }
  }

  //do a cumulative integration of the cost function (wrt time) using the trapezoidal method
  mpn_float cost = costFunction[0] + costFunction[size-1];
  for(int i(1); i<size-1; i++){
    cost += 2*costFunction[i];
  }
  
  return (dt/2.)*cost;
}

  /*
//Puts the nominal controlPath and the path in the given pointers
int nominalPath(Environment * e,Integrator * intgr,mpn_float** controlPath, mpn_float ** path,mpn_float * start,int steps,Integrator *& atCHI,MPNParams * params){
  mpn_float * current = start;
  int CHI = -1;
  Integrator * tmpPtr = NULL;
  if(params != NULL){
    CHI = ceil(static_cast<mpn_float>(params->controlHorizon/intgr->getDt()));
  }


  for(int i(0); i<steps; i++){

    //Check to see if we are close enough to the destination
    //  NOTE: This doesn't have to happen every point, maybe every 10
     
    if(params != NULL){
      if(gamma(e->goal,current,DIM) < pow(params->tolerance,2)){
	if(tmpPtr == NULL){
	  tmpPtr = intgr->copy();
	  tmpPtr->saveState();
	  atCHI = tmpPtr;
	}
	return i;
      }
      if(i == CHI){
	tmpPtr = intgr->copy();
	tmpPtr->saveState();
	atCHI = tmpPtr;
      }
    }

    // Euler integration step
    //   NOTE: these two function calls must be done in order and WILL change 
    //the internal state of each object.
     
    e->negatedGradient(current,controlPath[i]);
    intgr->step(current,controlPath[i],path[i]);
    current = path[i];

  }
  return steps;
}
*/

//Puts the nominal controlPath and the path in the given pointers
int nominalPath(Environment * e,Integrator * intgr,mpn_float* controlPath, mpn_float * path,mpn_float * start,int steps,Integrator *& atCHI,MPNParams * params){
  mpn_float * current = start;
  int CHI = -1;
  Integrator * tmpPtr = NULL;
  if(params != NULL){
    CHI = ceil(static_cast<mpn_float>(params->controlHorizon/intgr->getDt()));
  }


  for(int i(0); i<steps; i++){

    /*Check to see if we are close enough to the destination
      NOTE: This doesn't have to happen every point, maybe every 10
     */
    if(params != NULL){
      if(gamma(e->goal,current,DIM) < pow(params->tolerance,2)){
	if(tmpPtr == NULL){
	  tmpPtr = intgr->copy();
	  tmpPtr->saveState();
	  atCHI = tmpPtr;
	}
	return i;
      }
      if(i == CHI){
	tmpPtr = intgr->copy();
	tmpPtr->saveState();
	atCHI = tmpPtr;
      }
    }

    /* Euler integration step
       NOTE: these two function calls must be done in order and WILL change the
       internal state of each object.
     */
    e->negatedGradient(current,&controlPath[2*i]);
    intgr->step(current,&controlPath[2*i],&path[2*i]);
    current = &path[2*i];

  }
  return steps;
}

//Puts a sample controlPath and path into the given variables, given a set of parameters
int samplePath(Environment * e, Integrator * intgr,MPNParams * params,mpn_float* controlPath, mpn_float * path,mpn_float * start,int steps,Integrator *& atCHI){
  mpn_float * current = start,angPerturb,tmpSin,tmpCos;
  mpn_float * currentGrad;
  mpn_float currentPolyTime = params->currentTime/params->predictionHorizon;
  mpn_float polyDt = intgr->getDt()/params->predictionHorizon;
  int CHI = ceil(static_cast<mpn_float>(params->controlHorizon/intgr->getDt()));
  mpn_float tmpGrad[2];
  Integrator * tmpPtr = NULL;

  e->negatedGradient(current,&controlPath[0]);
  intgr->step(current,&controlPath[0],&path[0]);
  current = &path[0];

  for(int i(1); i<steps; i++){

    /*Check to see if we are close enough to the destination
      NOTE: This doesn't have to happen every point, maybe every 10
     */
    if(gamma(current,e->goal,DIM) < pow(params->tolerance,2)){
      if(tmpPtr == NULL){
	tmpPtr = intgr->copy();
	tmpPtr->saveState();
	atCHI = tmpPtr;
      }
      return i;
    }
    if(i == CHI){
      tmpPtr = intgr->copy();
      tmpPtr->saveState();
      atCHI = tmpPtr;
    }
    
    /*
      Integration step
     */
    currentGrad = &controlPath[2*i];
    e->negatedGradient(current,currentGrad);//store nominal gradient
    currentPolyTime += polyDt;
    
    //angPerturb = anglePerturb(params,currentPolyTime);

    angPerturb = 0;
    for(unsigned int j(0); j<params->nLegendrePolys; j++){
      if(params->controlParameters[j] != 0)
	angPerturb += params->controlParameters[j]*
	  alglib::legendrecalculate(j,static_cast<double>(currentPolyTime));
    }
    angPerturb*=(M_PI);

    tmpCos = cos(angPerturb);
    tmpSin = sin(angPerturb);
    
    //store new gradient
    tmpGrad[0] = currentGrad[0]*tmpCos - currentGrad[1]*tmpSin;
    tmpGrad[1] = currentGrad[1]*tmpCos + currentGrad[0]*tmpSin;
    currentGrad[0] = tmpGrad[0];
    currentGrad[1] = tmpGrad[1];

    intgr->step(current,currentGrad,&path[2*i]);
    current = &path[2*i];

  }
  return steps;
}

bool generateBestPath(Environment * e, MPNParams * params, Integrator *& intgr, mpn_float *&bestPath, mpn_float *& bestControl, int & steps, int & controlHorizonIndex,mpn_float * start){
  
  //ln(a) = log(a)/log(e) -> ln(a)/ln(b) = log(a)/log(b)
  mpn_float nSamples = log(1/(params->confidence))/log(1/(1-params->level));
  //std::cout << "samples: " << nSamples << std::endl;
  steps = ceil(static_cast<mpn_float>(params->predictionHorizon)/intgr->getDt());
  controlHorizonIndex = ceil(static_cast<mpn_float>(params->controlHorizon/
						 intgr->getDt()));
  
  mpn_float startCost = e->potentialField(start);
  
  mpn_float * nominal = allocatePoints(steps);
  mpn_float * nominalControl = allocatePoints(steps);
  Integrator * optimalEndIntegrator,* tempPtr = intgr->copy();
  int bestSteps=nominalPath(e,intgr,nominalControl,nominal,start,steps,optimalEndIntegrator,params);
  int bestCHI=bestSteps < controlHorizonIndex ? bestSteps : controlHorizonIndex;

  //initialize with nominal values
  mpn_float * optimalPath = allocatePoints(steps);
  for(int i(0); i<2*steps; i+=2){
    optimalPath[i] = nominal[i];
    optimalPath[i+1] = nominal[i+1];
  }
  mpn_float * optimalControl = allocatePoints(steps);
  for(int i(0); i<2*steps; i+=2){
    optimalControl[i] = nominalControl[i];
    optimalControl[i+1] = nominalControl[i+1];
  }

  mpn_float incrCost = incrementalCost(e,params,nominal,nominalControl,
				    intgr->getDt(),steps);
  mpn_float termCost = terminalCost(e,nominal,steps);
  mpn_float optimalCost = incrCost + termCost;
  
  
  mpn_float optimalParams[params->nLegendrePolys];
  for(unsigned int i(0); i<params->nLegendrePolys;i++){optimalParams[i]=0;}
  
  mpn_float currentCost,currentTerminal,currentControlHorizonCost,currentFinalOri;
  int currSteps,currCHI;
  mpn_float * currentPath = allocatePoints(steps);
  mpn_float * currentControlPath = allocatePoints(steps);
  
  int acceptedSoFar = 1,tries = 0;
  mpn_float tmpl = 1.0/params->nLegendrePolys;
  mpn_float gammaCost = gamma(start,e->goal,2);
  do{
    intgr->reset();
    //generate control inputs on +/- 1/(number of inputs)
    for(unsigned int i(0); i<params->nLegendrePolys; i++){
      params->controlParameters[i] = (2.0*tmpl)*(rand()*(1.0/(RAND_MAX + 1.0))) - tmpl;
    }
    delete tempPtr;
    currSteps=samplePath(e,intgr,params,currentControlPath,currentPath,start,steps,tempPtr);
    currCHI=currSteps < controlHorizonIndex ? currSteps : controlHorizonIndex;
    currentControlHorizonCost = terminalCost(e,currentPath,currCHI);
    
    //If we made progress toward the goal, count this path
    if(currentControlHorizonCost - startCost <= gammaCost ){
      currentTerminal = terminalCost(e,currentPath,currSteps);
      
      //Only calculate incremental cost if there is a chance this path is 
      //optimal
      if(currentTerminal < optimalCost){
	currentCost = currentTerminal + incrementalCost(e,params,currentPath,currentControlPath,intgr->getDt(),currSteps);
	acceptedSoFar++;
	//If we found a better path, save it
	if(currentCost < optimalCost){
	  bestSteps = currSteps;
	  bestCHI = currCHI;
	  optimalCost = currentCost;
	  //Copy values into optimalPath
	  for(int i(0); i<2*currSteps; i+=2){
	    optimalPath[i] = currentPath[i];
	    optimalPath[i+1] = currentPath[i+1];
	  }
	  //Copy values into optimalControl
	  for(int i(0); i<2*currSteps; i+=2){
	    optimalControl[i] = currentControlPath[i];
	    optimalPath[i+1] = currentControlPath[i+1];
	  }
	  //Copy parameters into optimalParams
	  for(unsigned int i(0); i<params->nLegendrePolys; i++){
	    optimalParams[i] = params->controlParameters[i];
	  }
	  //Copy Integrator end state into optimalEndIntegrator
	  //optimalEndIntegrator = *intgr;
	  delete optimalEndIntegrator;
	  //optimalEndIntegrator = intgr->copy();
	  optimalEndIntegrator = tempPtr->copy();

	  //memcpy(&optimalEndIntegrator,intgr,sizeof(Unicycle));
	}
      }
    }
    tries++;
  }while(acceptedSoFar <= nSamples && tries < MAX_TRIES);
  //std::cout << "Tries: " << tries << std::endl;
  //std::cout << "Accepted so far: " << acceptedSoFar << std::endl;
  
  bestPath = optimalPath;
  bestControl = optimalControl;
  for(int i(2*(bestSteps+1)); i<2*steps; i+=2){
    bestPath[i] = 0;
    bestPath[i+1] = 0;
    bestControl[i] = 0;
    bestControl[i+1] = 0;
  }
  steps = bestSteps;

  bool arrived = bestCHI < controlHorizonIndex;
  controlHorizonIndex = bestCHI;
  for(unsigned int i(0); i<params->nLegendrePolys; i++){
    params->controlParameters[i] = optimalParams[i];
  }

  intgr = optimalEndIntegrator;

  cleanupPoints(nominal);
  cleanupPoints(currentPath);
  cleanupPoints(nominalControl);
  cleanupPoints(currentControlPath);

  //std::cout << "Alleged angle: " << dynamic_cast<Unicycle *>(optimalEndIntegrator)->currTheta << std::endl;
  //std::cout << "Angle from x: " << atan2(bestPath[bestCHI+1][1]-bestPath[bestCHI][1],bestPath[bestCHI+1][0]-bestPath[bestCHI][0]) << std::endl;
  //std::cout << "Angle from dx: " << atan2(bestControl[bestCHI][1],bestControl[bestCHI][0]) << std::endl;

  //std::cout << "Cost: " << optimalCost << std::endl;

  return arrived;
}
/*
bool generateBestPath(Environment * e, MPNParams * params, Integrator *& intgr, mpn_float **&bestPath, mpn_float **& bestControl, int & steps, int & controlHorizonIndex,mpn_float * start){
  
  //ln(a) = log(a)/log(e) -> ln(a)/ln(b) = log(a)/log(b)
  mpn_float nSamples = log(1/(params->confidence))/log(1/(1-params->level));
  //std::cout << "samples: " << nSamples << std::endl;
  steps = ceil(static_cast<mpn_float>(params->predictionHorizon)/intgr->getDt());
  controlHorizonIndex = ceil(static_cast<mpn_float>(params->controlHorizon/
						 intgr->getDt()));
  
  mpn_float startCost = e->potentialField(start);
  
  mpn_float ** nominal = allocatePoints(steps);
  mpn_float ** nominalControl = allocatePoints(steps);
  Integrator * optimalEndIntegrator,* tempPtr = intgr->copy();
  int bestSteps=nominalPath(e,intgr,nominalControl,nominal,start,steps,optimalEndIntegrator,params);
  int bestCHI=bestSteps < controlHorizonIndex ? bestSteps : controlHorizonIndex;

  //initialize with nominal values
  mpn_float ** optimalPath = allocatePoints(steps);
  for(int i(0); i<steps; i++){
    optimalPath[i][0] = nominal[i][0];
    optimalPath[i][1] = nominal[i][1];
  }
  mpn_float ** optimalControl = allocatePoints(steps);
  for(int i(0); i<steps; i++){
    optimalControl[i][0] = nominalControl[i][0];
    optimalControl[i][1] = nominalControl[i][1];
  }

  mpn_float incrCost = incrementalCost(e,params,nominal,nominalControl,
				    intgr->getDt(),steps);
  mpn_float termCost = terminalCost(e,nominal,steps);
  mpn_float optimalCost = incrCost + termCost;
  
  
  mpn_float optimalParams[params->nLegendrePolys];
  for(unsigned int i(0); i<params->nLegendrePolys;i++){optimalParams[i]=0;}
  
  mpn_float currentCost,currentTerminal,currentControlHorizonCost,currentFinalOri;
  int currSteps,currCHI;
  mpn_float ** currentPath = allocatePoints(steps);
  mpn_float ** currentControlPath = allocatePoints(steps);
  
  int acceptedSoFar = 1,tries = 0;
  mpn_float tmpl = 1.0/params->nLegendrePolys;
  mpn_float gammaCost = gamma(start,e->goal,2);
  do{
    intgr->reset();
    //generate control inputs on +/- 1/(number of inputs)
    for(unsigned int i(0); i<params->nLegendrePolys; i++){
      params->controlParameters[i] = (2.0*tmpl)*(rand()*(1.0/(RAND_MAX + 1.0))) - tmpl;
    }
    delete tempPtr;
    currSteps=samplePath(e,intgr,params,currentControlPath,currentPath,start,steps,tempPtr);
    currCHI=currSteps < controlHorizonIndex ? currSteps : controlHorizonIndex;
    currentControlHorizonCost = terminalCost(e,currentPath,currCHI);
    
    //If we made progress toward the goal, count this path
    if(currentControlHorizonCost - startCost <= gammaCost ){
      currentTerminal = terminalCost(e,currentPath,currSteps);
      
      //Only calculate incremental cost if there is a chance this path is 
      //optimal
      if(currentTerminal < optimalCost){
	currentCost = currentTerminal + incrementalCost(e,params,currentPath,currentControlPath,intgr->getDt(),currSteps);
	acceptedSoFar++;
	//If we found a better path, save it
	if(currentCost < optimalCost){
	  bestSteps = currSteps;
	  bestCHI = currCHI;
	  optimalCost = currentCost;
	  //Copy values into optimalPath
	  for(int i(0); i<currSteps; i++){
	    optimalPath[i][0] = currentPath[i][0];
	    optimalPath[i][1] = currentPath[i][1];
	  }
	  //Copy values into optimalControl
	  for(int i(0); i<currSteps; i++){
	    optimalControl[i][0] = currentControlPath[i][0];
	    optimalControl[i][1] = currentControlPath[i][1];
	  }
	  //Copy parameters into optimalParams
	  for(unsigned int i(0); i<params->nLegendrePolys; i++){
	    optimalParams[i] = params->controlParameters[i];
	  }
	  //Copy Integrator end state into optimalEndIntegrator
	  //optimalEndIntegrator = *intgr;
	  delete optimalEndIntegrator;
	  //optimalEndIntegrator = intgr->copy();
	  optimalEndIntegrator = tempPtr->copy();

	  //memcpy(&optimalEndIntegrator,intgr,sizeof(Unicycle));
	}
      }
    }
    tries++;
  }while(acceptedSoFar <= nSamples && tries < MAX_TRIES);
  //std::cout << "Tries: " << tries << std::endl;
  //std::cout << "Accepted so far: " << acceptedSoFar << std::endl;
  
  bestPath = optimalPath;
  bestControl = optimalControl;
  for(int i(bestSteps+1); i<steps; i++ ){
    delete [] bestPath[i];
    delete [] bestControl[i];
  }
  steps = bestSteps;

  bool arrived = bestCHI < controlHorizonIndex;
  controlHorizonIndex = bestCHI;
  for(unsigned int i(0); i<params->nLegendrePolys; i++){
    params->controlParameters[i] = optimalParams[i];
  }

  intgr = optimalEndIntegrator;

  cleanupPoints(nominal,steps);
  cleanupPoints(currentPath,steps);
  cleanupPoints(nominalControl,steps);
  cleanupPoints(currentControlPath,steps);

  //std::cout << "Alleged angle: " << dynamic_cast<Unicycle *>(optimalEndIntegrator)->currTheta << std::endl;
  //std::cout << "Angle from x: " << atan2(bestPath[bestCHI+1][1]-bestPath[bestCHI][1],bestPath[bestCHI+1][0]-bestPath[bestCHI][0]) << std::endl;
  //std::cout << "Angle from dx: " << atan2(bestControl[bestCHI][1],bestControl[bestCHI][0]) << std::endl;

  //std::cout << "Cost: " << optimalCost << std::endl;

  return arrived;
}
*/
