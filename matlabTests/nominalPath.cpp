#include "mex.h"
#include "Environment.h"
#include "MPN2D.h"
#include "Build.h"
#include <string.h>
#include <iostream>

#define DEFAULT_FILE "lab.cfg"
/*
  Go-between from matlab to MPN2D::nominalPath for visualization

  Input arguments: [x,y] start, precision(dt), steps, (optional filename)
  Output arguments: row vector of x values, row vector of y values (optional 
  row vector of dx values, row vector of dy values)

  Example usage in matlab:
  [x,y] = nominalPath([.3,.3],.01,100,'default.cfg')
  plot(x,y)

*/
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) {
  //Check for correct function syntax
  if(!(nrhs == 3 || nrhs == 4))
    mexErrMsgTxt("Incorrect number of input arguments");
  if(!(nlhs == 2 || nlhs == 4))
    mexErrMsgTxt("Incorrect number of output arguments");

  //Get start position and check for correct dimensions
  if(mxGetM(prhs[0]) != 1 || !(mxGetN(prhs[0]) == 2 ))
    mexErrMsgTxt("Start position must be in [x,y] form");
  double start[2] = {mxGetPr(prhs[0])[0],mxGetPr(prhs[0])[1]};

  //Get precision, steps
  double dt = *mxGetPr(prhs[1]);
  unsigned int steps = static_cast<int>(*mxGetPr(prhs[2]));

  //Initialize environment
  Environment * e;
  Integrator * intgr;
  char filename[256];
  if(nrhs == 4){
    mxGetString(prhs[3],filename,mxGetN(prhs[3])+1);
  }
  else{
    strcpy(filename,DEFAULT_FILE);
  }

  buildEnvironment(filename,e,2);
  buildIntegrator(filename,intgr,2,dt);
  
  //Allocate space for the answer
  plhs[0] = mxCreateDoubleMatrix(1,steps,mxREAL);
  plhs[1] = mxCreateDoubleMatrix(1,steps,mxREAL);
  if(nlhs == 4){
    plhs[2] = mxCreateDoubleMatrix(1,steps,mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1,steps,mxREAL);    
  }
  
  //Calculate the nominal path
  double ** nominal = allocatePoints(steps);
  double ** controlPath = allocatePoints(steps);

  nominalPath(e,intgr,controlPath,nominal,start,steps);


  //Copy the answer into the matlab vectors for output
  for(int i(0); i<steps; i++){

    //printf("nominal: %f,%f\n",nominal[i][0],nominal[i][1]);
    
    mxGetPr(plhs[0])[i] = nominal[i][0];
    mxGetPr(plhs[1])[i] = nominal[i][1];

    if(nlhs == 4){
      mxGetPr(plhs[2])[i] = controlPath[i][0];
      mxGetPr(plhs[3])[i] = controlPath[i][1];      
    }

  }

  cleanupPoints(nominal,steps);
  cleanupPoints(controlPath,steps);
}
