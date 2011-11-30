#include "datatypes.h"
#include "Build.h"
#include "MPNParams.h"
#include <libconfig.h++>
#include "Environment.h"
#include "Integrator.h"
#include "Unicycle.h"

#include <iostream>

using namespace libconfig;

void buildAll(const char * filename,Environment * & env, Integrator * & intg,
	      MPNParams * & mp,mpn_float stepTime){
  Config c;
  c.readFile(filename);

  buildMPNParams(&c,mp);
  if(mp == NULL){
    return;
  }
  buildEnvironment(&c,env,mp->wsDim);
  if(env == NULL)
    return;
  buildIntegrator(&c,intg,mp->wsDim,stepTime);
  if(intg == NULL)
    return;
}

void buildEnvironment(const char * filename,Environment * & env,int wsdim){
  Config c;
  c.readFile(filename);
  buildEnvironment(&c,env,wsdim);
}
void buildIntegrator(const char * filename,Integrator * & intg,int wsdim,
		     mpn_float stepTime){
  Config c;
  c.readFile(filename);
  buildIntegrator(&c,intg,wsdim,stepTime);
}
void buildMPNParams(const char * filename,MPNParams * & mp){
  Config c;
  c.readFile(filename);
  buildMPNParams(&c,mp);
}

void buildEnvironment(Config * cfg,Environment * & env,int wsdim){
  cfg->setAutoConvert(true);

  if(cfg->exists(ENVIRONMENT)){
    env = new Environment(cfg->lookup(ENVIRONMENT),wsdim);
    return;
  }

  else if(cfg->exists(DIPLOAR_ENVIRONMENT)){
    env = new DipolarEnvironment(cfg->lookup(DIPLOAR_ENVIRONMENT));
    return;
  }

  else{
    env = NULL;
    return;
  }

}

void buildIntegrator(Config * cfg,Integrator *& intg,int wsdim,mpn_float stepTime){
  cfg->setAutoConvert(true);

  if(cfg->exists(UNICYCLE_INTEGRATOR)){
    intg = new Unicycle(cfg->lookup(UNICYCLE_INTEGRATOR),stepTime,wsdim);
    return;
  }
  else{
    intg = new Integrator(stepTime,wsdim);
    return;
  }

}

void buildMPNParams(Config * cfg,MPNParams * & mp){
  cfg->setAutoConvert(true);

  if(cfg->exists(MPN_PARAMETERS)){
    mp = new MPNParams;
    Setting & group = cfg->lookup(MPN_PARAMETERS);
    mp->wsDim = group["dimension"];
    mp->tolerance = group["tolerance"];
    mp->nLegendrePolys = group["num_legendre_polys"];
    mp->confidence = group["confidence"];
    mp->level = group["level"];
    mp->predictionHorizon = group["prediction_horizon"];
    mp->controlHorizon = group["control_horizon"];

    if(group.exists("start_time")){
      mp->currentTime = group["start_time"];
    }
    else{
      mp->currentTime = 0.0;
    }

    if(group.exists("cost_weights")){
      Setting & cw = group["cost_weights"];
      for(int i(0); i<3; i++){
	mp->costWeights[i] = cw[i];
      }
    }
    else{
      for(int i(0); i<3; i++)
	mp->costWeights[i] = 1;
    }
    
    mp->controlParameters = new mpn_float[mp->nLegendrePolys];
    if(group.exists("control")){
      Setting & cp = group["control"];
      for(int i(0); i<mp->nLegendrePolys; i++){
	mp->controlParameters[i] = cp[i];
      }
    }
    else{
      for(int i(0); i<mp->nLegendrePolys; i++){
	mp->controlParameters[i] = 0;
      }
    }
    return;
  }

  else{
    mp = NULL;
    return;
  }
}
