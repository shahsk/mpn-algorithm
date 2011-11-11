#include "mex.h"
#include "Build.h"
#include "Environment.h"
#include "Obstacle.h"
#include <string.h>

#define DEFAULT_FILE "lab.cfg"

/*
  Gets the obstacle positions out of the config file and into matlab

  Input arguments: (optional filename)
  Output arguments: row vector of x, row vector of y, row vector of radii

 */
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) {
  if(nrhs > 1)
    mexErrMsgTxt("Incorrect number of input arguments");
  if(nlhs != 3)
    mexErrMsgTxt("Incorrect number of output arguments");

  //Initialize environment
  Environment * e;
  char filename[256];
  if(nrhs == 1){
    mxGetString(prhs[0],filename,256);
  }
  else{
    strcpy(filename,DEFAULT_FILE);
  }

  buildEnvironment(filename,e,2);

  //Allocate space for the answer
  unsigned int nObs = e->obstacles.size();
  plhs[0] = mxCreateDoubleMatrix(1,nObs,mxREAL);
  plhs[1] = mxCreateDoubleMatrix(1,nObs,mxREAL);
  plhs[2] = mxCreateDoubleMatrix(1,nObs,mxREAL);
  
  //Populate the answer array with the positions/radii
  for(unsigned int i(0); i<nObs; i++ ){
    mxGetPr(plhs[0])[i] = e->obstacles[i]->pos[0];
    mxGetPr(plhs[1])[i] = e->obstacles[i]->pos[1];
    mxGetPr(plhs[2])[i] = e->obstacles[i]->radius;
  }

}
