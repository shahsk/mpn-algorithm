#include "Environment.h"

#define POTENTIAL 1.3

Environment * setupEnv(double * goal,double rad){
  Environment * e = new Environment(goal,POTENTIAL,rad);
  
  //Obstacle setup
  double obPos1[2] = {rad*.1,rad*.1};
  double obPos2[2] = {rad*.1,rad*.3};
  e->obstacles.push_back(Obstacle(obPos1,.2*rad));
  e->obstacles.push_back(Obstacle(obPos2,.2*rad));
  
  return e;
}

