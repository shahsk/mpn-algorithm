#include "mex.h"
#include "Environment.h"
#include "MPN2D.h"
#include "configure.h"
#include <stdlib.h>
#include <time.h>

/*
  Go-between from matlab to MPN2D::generateBestPath for visualization

  Input arguments: [x,y,(theta)] start, precision(dt), (optional currentTime,filename)
  Output arguments: row vector of x values, row vector of y values, control 
  horizon index, (optional dx, dy)

  Example usage in matlab:
  [x,y] = bestPath([.1,.1],.01)


*/
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ]) {
  //Check for correct function syntax
  if(!(nrhs == 2 || nrhs == 3 || nrhs == 4) )
    mexErrMsgTxt("Incorrect number of input arguments");
  if(!(nlhs == 3 || nlhs == 5))
    mexErrMsgTxt("Incorrect number of output arguments");

  //Get start position and check for correct dimensions
  if(mxGetM(prhs[0]) != 1 || !(mxGetN(prhs[0]) == 2 || mxGetN(prhs[0]) == 3 ))
    mexErrMsgTxt("Start position must be in [x,y,theta] form");

  double start[2] = {mxGetPr(prhs[0])[0],mxGetPr(prhs[0])[1]};
  double startOri;

  if(mxGetN(prhs[0]) == 3)
    startOri = mxGetPr(prhs[0])[2];
  else
    startOri = 0;

  //Get precision
  double dt = *mxGetPr(prhs[1]);

  //Initialize environment
  Environment * e;
  MPNParams * mp;
  if(nrhs == 4){
    char filename[256];
    mxGetString(prhs[3],filename,mxGetN(prhs[3])+1);
    //mexPrintf(filename);
    configure(filename,e,mp);
  }
  else{
    configure(e,mp);
  }
  
  //Adjust current time if necessary
  if(nrhs == 3 || nrhs == 4)
    mp->currentTime = *mxGetPr(prhs[2]);

  //Calculate a sample path
  srand(time(NULL));
  double ** path, ** control,finalOri;
  int steps,controlIndex;
  generateBestPath(*e,*mp,path,control,steps,controlIndex,start,startOri,dt,finalOri);

  //Allocate space for the answer
  plhs[0] = mxCreateDoubleMatrix(1,steps,mxREAL);
  plhs[1] = mxCreateDoubleMatrix(1,steps,mxREAL);
  plhs[2] = mxCreateDoubleMatrix(1,1,mxREAL);
  if(nlhs == 5){
    plhs[3] = mxCreateDoubleMatrix(1,steps,mxREAL);
    plhs[4] = mxCreateDoubleMatrix(1,steps,mxREAL);
  }  
  //Copy the answer into the matlab vectors for output
  for(int i(0); i<steps; i++){
    mxGetPr(plhs[0])[i] = path[i][0];
    mxGetPr(plhs[1])[i] = path[i][1];
    if(nlhs == 5){
      mxGetPr(plhs[3])[i] = path[i][0];
      mxGetPr(plhs[4])[i] = path[i][1];
    }
  }
  *mxGetPr(plhs[2]) = static_cast<double>(controlIndex + 1);
  cleanupPoints(path,steps);
}
