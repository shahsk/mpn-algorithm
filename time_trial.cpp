#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include "Build.h"
#include "MPN2D.h"
#include "MPNParams.h"
#include "Environment.h"
#include "Integrator.h"

#define CONFIG_FILE "lab.cfg"
#define STEP .01
#define STARTX .9 //Starting point is [STARTX*radius,STARTY*radius]
#define STARTY .9

int main(){
  Environment * env;
  Integrator * bot;
  MPNParams * mp;
  
  //Use config file to create the object we need
  buildAll(CONFIG_FILE,env,bot,mp,STEP);

  //Seed rand so that we get the same result every time
  srand(0);

  /*
  int nSamples = ceil(log(1/(mp->confidence))/
				  log(1/(1-mp->level)));
  int steps = ceil(static_cast<double>(mp->predictionHorizon)/
		   bot->getDt());
  */

  double ** bestPath,** bestControl, start[2];
  int steps,CHI;
  start[0] = STARTX*env->radius;
  start[1] = STARTY*env->radius;

  clock_t start_t = clock();

  generateBestPath(env,mp,bot,bestPath,bestControl,steps,CHI,start);

  clock_t end_t = clock();

  printf("Cycles: %u \n",end_t-start_t);
}
