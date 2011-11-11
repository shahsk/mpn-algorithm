#include "mex.h"
#include "Build.h"
#include "Environment.h"
#include <string.h>

#define DEFAULT_FILE "lab.cfg"

/*
  Go-between from matlab to environment.potentialField for visualization

  Input arguments: matrix of x values, matrix of y values, (optional filename)
  Output arguments: matrix of z values for each [x,y] point

  Example usage in matlab:
  [xa,ya] = meshgrid(0:.01:1,0:.01:1);
  z = potentialField(xa,ya,'default.cfg');
  surf(xa,ya,z)

*/
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) {
  //Check for correct function syntax
  if(!(nrhs == 2 || nrhs == 3))
    mexErrMsgTxt("Incorrect number of input arguments");
  if(nlhs != 1)
    mexErrMsgTxt("Incorrect number of output arguments");

  //Get matrix dimensions and check for matching
  unsigned int rows,cols;
  rows = mxGetM(prhs[0]);
  cols = mxGetN(prhs[0]);
  if(rows != mxGetM(prhs[1]) || cols != mxGetN(prhs[1]) )
    mexErrMsgTxt("X and Y matrix dimensions must agree");

  //Initialize environment
  Environment * e;
  char filename[256];
  if(nrhs == 3){
    mxGetString(prhs[2],filename,mxGetN(prhs[2])+1);
  }
  else{
    strcpy(filename,DEFAULT_FILE);
  }
  
  buildEnvironment(filename,e,2);

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
      mxGetPr(plhs[0])[index] = e->potentialField(tmpPoint);
    }
  }
}
