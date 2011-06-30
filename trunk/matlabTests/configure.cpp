#include <libconfig.h++>
#include <iostream>
#include "Environment.h"
#include "Obstacle.h"
#include "MPN2D.h"
#include <string.h>

#include "configure.h"

#define DEFAULTFILE "default.cfg"

using namespace libconfig;
using namespace std;

void configure(char file[],Environment * & e,MPNParams * & mp){

  Config c;
  c.readFile(file);

  double radius = c.lookup("environment.radius");
  double pot_param = c.lookup("environment.potential_parameter");
  double goal[2] = {c.lookup("environment.destination")[0],
		 c.lookup("environment.destination")[1]};

  const char * type;
  if(c.lookupValue("environment.type",type)){
    if(strcmp(type,"dipolar") == 0 ){
      double goalOri = c.lookup("environment.goal_orientation");
      double epsilon = c.lookup("environment.epsilon");
      e  = new DipolarEnvironment(goal,pot_param,radius,epsilon,goalOri);
    }
  }
  else{
    e = new Environment(goal,pot_param,radius);
  }

  if(c.exists("environment.obstacles")){
    double rad,pos[2];
    Setting & obs = c.lookup("environment.obstacles");
    for(int i(0); i<obs.getLength(); i++){
      rad = obs[i]["radius"];
      pos[0] = obs[i]["position"][0];
      pos[1] = obs[i]["position"][1];
      e->obstacles.push_back(Obstacle(pos,rad));
    }
  }

  //Initialize mandatory parameters first
  mp = new MPNParams;
  mp->nLegendrePolys = c.lookup("mpn_parameters.num_legendre_polys");
  mp->confidence = c.lookup("mpn_parameters.confidence");
  mp->level = c.lookup("mpn_parameters.level");
  mp->predictionHorizon = c.lookup("mpn_parameters.prediction_horizon");
  mp->controlHorizon = c.lookup("mpn_parameters.control_horizon");
  mp->currentTime = c.lookup("mpn_parameters.start_time");

  if(c.exists("mpn_parameters.cost_weights")){
    Setting & cw = c.lookup("mpn_parameters.cost_weights");
    for(int i(0); i<3; i++){
      mp->costWeights[i] = cw[i];
    }
  }
  else{
    for(int i(0); i<3; i++)
      mp->costWeights[i] = 0;
  }

  mp->controlParameters = new double[mp->nLegendrePolys];
  if(c.exists("mpn_parameters.control")){
    Setting & cp = c.lookup("mpn_parameters.control");
    for(int i(0); i<mp->nLegendrePolys; i++){
      mp->controlParameters[i] = cp[i];
    }
  }
  else{
    for(int i(0); i<mp->nLegendrePolys; i++)
      mp->controlParameters[i] = 0;
  }

}

void configure(Environment * & e,MPNParams * & mp){
  char filename[] = DEFAULTFILE;
  configure(filename,e,mp);
}
