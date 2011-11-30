#ifndef __MPNPARAMS_H
#define __MPNPARAMS_H

#include "datatypes.h"

#define MPN_PARAMETERS "mpn_parameters"

/*
  Config grammar example:

  mpn_parameters: 
  {
    #Required
    dimension = 2;
    tolerance = .1;
    num_legendre_polys = 5;
    confidence = .05;
    level = .05;
    prediction_horizon = 10.;
    control_horizon = 2.;

    #Optional
    start_time = 0.;
    cost_weights = [1.,1.,1.];
    control = [1.,1.,1.,1.,1.]; #Length must be equal to num_legendre_polys
  };
  
*/
struct MPNParams{
  unsigned int wsDim;
  unsigned int nLegendrePolys;
  mpn_float * controlParameters;
  mpn_float confidence,level,predictionHorizon,controlHorizon,currentTime;
  mpn_float costWeights[3];
  mpn_float tolerance;
};

#endif
