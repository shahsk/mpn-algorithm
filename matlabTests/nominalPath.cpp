#include "mex.h"
#include "setupEnv.h"
#include "Environment.h"
#include "MPN2D.h"

/*
  Go-between from matlab to MPN2D::nominalPath for visualization
  To customize the environment, see setupEnv.cpp

  Input arguments: [x,y] start, [x,y] goal, radius, precision(dt), steps
  Output arguments: row vector of x values, row vector of y values (optional 
  row vector of dx values, row vector of dy values)

  Example usage in matlab:
  >>> [x,y] = nominalPath([.1,.5],[.5,.5],1,.01,100);
  >>> plot(x,y)
  Example 2:
  >>> [x,y,dx,dy] = nominalPath([.1,.5],[.5,.5],1,.01,100);
  >>> plot(dx,dy)



*/
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) {
  //Check for correct function syntax
  if(nrhs != 5)
    mexErrMsgTxt("Incorrect number of input arguments");
  if(!(nlhs == 2 || nlhs == 4))
    mexErrMsgTxt("Incorrect number of output arguments");

  //Get start position and check for correct dimensions
  if(mxGetM(prhs[0]) != 1 || mxGetN(prhs[0]) != 2)
    mexErrMsgTxt("Start position must be in [x,y] form");
  double start[2] = {mxGetPr(prhs[0])[0],mxGetPr(prhs[0])[1]};

  //Get goal position and check for correct dimensions
  if(mxGetM(prhs[1]) != 1 || mxGetN(prhs[1]) != 2)
    mexErrMsgTxt("Goal position must be in [x,y] form");
  double goal[2] = {mxGetPr(prhs[1])[0],mxGetPr(prhs[1])[1]};
  
  //Get radius, precision, steps
  double radius = *mxGetPr(prhs[2]);
  double dt = *mxGetPr(prhs[3]);
  unsigned int steps = static_cast<int>(*mxGetPr(prhs[4]));

  //Initialize environment
  Environment e = *setupEnv(goal,radius);

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
  nominalPath(e,controlPath,nominal,start,dt,steps);

  //Copy the answer into the matlab vectors for output
  for(int i(0); i<steps; i++){
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
