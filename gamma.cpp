#include "datatypes.h"
#include "gamma.h"
#include <math.h>

#ifdef NEON
#include <arm_neon.h>
#endif

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

#if (defined SINGLE_PRECISION && defined ARM)
mpn_float gamma(mpn_float*v1,mpn_float*v2,unsigned int deprecated){
  float32x2_t a = vld1_f32(v1);
  float32x2_t b = vld1_f32(v2);
  a = vsub_f32(a,b);
  a = vmul_f32(a,a);
  return vget_lane_f32(a,0)+vget_lane_f32(a,1);
}
#else
mpn_float gamma(mpn_float*v1,mpn_float*v2,unsigned int deprecated){
  mpn_float a = (v1[0]-v2[0]);
  mpn_float b = (v1[1]-v2[1]);
  return a*a + b*b;
}
#endif

#endif
