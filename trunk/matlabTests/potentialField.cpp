#include "mex.h"
#include "setupEnv.h"
#include "Environment.h"

/*
  Go-between from matlab to environment.potentialField for visualization
  To customize the environment, see setupEnv.cpp

  Input arguments: matrix of x values, matrix of y values, 
                   [x,y] goal position, radius
  Output arguments: matrix of z values for each [x,y] point

  Example usage in matlab:
  >>> [x,y] = meshgrid(0:.01:1,0:.01:1);
  >>> z = potentialField(x,y,[.5,.5],1);
  >>> surf(x,y,z)
*/
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) {
  //Check for correct function syntax
  if(nrhs != 4)
    mexErrMsgTxt("Incorrect number of input arguments");
  if(nlhs != 1)
    mexErrMsgTxt("Incorrect number of output arguments");

  //Get matrix dimensions and check for matching
  unsigned int rows,cols;
  rows = mxGetM(prhs[0]);
  cols = mxGetN(prhs[0]);
  if(rows != mxGetM(prhs[1]) || cols != mxGetN(prhs[1]) )
    mexErrMsgTxt("X and Y matrix dimensions must agree");

  //Get goal position and check for correct dimensions
  if(mxGetM(prhs[2]) != 1 || mxGetN(prhs[2]) != 2)
    mexErrMsgTxt("Goal position must be in [x,y] form");
  double goal[2] = {mxGetPr(prhs[2])[0],mxGetPr(prhs[2])[1]};
  
  //Get radius
  double radius = *mxGetPr(prhs[3]);

  //Initialize environment
  Environment e = *setupEnv(goal,radius);

  //Allocate space for the answer
  plhs[0] = mxCreateDoubleMatrix(rows,cols,mxREAL);
  

  //Populate the answer array with the value of the potential field
  double tmpPoint[2];
  int index;
  for(unsigned int r(0); r<rows; r++){
    for(unsigned int c(0); c<cols; c++){
      index = r + rows*c;
      tmpPoint[0] = mxGetPr(prhs[0])[index];
      tmpPoint[1] = mxGetPr(prhs[1])[index];
      mxGetPr(plhs[0])[index] = e.potentialField(tmpPoint);
    }
  }
}
