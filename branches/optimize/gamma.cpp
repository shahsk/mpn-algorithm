#include "datatypes.h"
#include "gamma.h"
#include <math.h>

#ifndef DIM2

//Returns the n-dimensional distance SQUARED between 2 points
mpn_float gamma(mpn_float * p1, mpn_float * p2, unsigned int dim){
	mpn_float mag = 0;
	for(unsigned int i(0); i<dim; i++)
		mag += pow(p1[i] - p2[i],2);
	return mag;
}

//Returns the n-dimensional distance SQUARED to the origin
mpn_float gamma(mpn_float* p,unsigned int dim){
	mpn_float mag = 0;
	for(unsigned int i(0); i<dim; i++)
		mag += pow(p[i],2);
	return mag;
}

#else

mpn_float gamma(mpn_float*v,unsigned int deprecated){
  return v[0]*v[0] + v[1]*v[1];
}

mpn_float gamma(mpn_float*v1,mpn_float*v2,unsigned int deprecated){
  mpn_float a = (v1[0]-v2[0]);
  mpn_float b = (v1[1]-v2[1]);
  return a*a + b*b;
}

#endif
