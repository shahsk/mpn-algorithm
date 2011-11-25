#include "gamma.h"
#include <math.h>

#ifndef DIM2

//Returns the n-dimensional distance SQUARED between 2 points
double gamma(double * p1, double * p2, unsigned int dim){
	double mag = 0;
	for(unsigned int i(0); i<dim; i++)
		mag += pow(p1[i] - p2[i],2);
	return mag;
}

//Returns the n-dimensional distance SQUARED to the origin
double gamma(double* p,unsigned int dim){
	double mag = 0;
	for(unsigned int i(0); i<dim; i++)
		mag += pow(p[i],2);
	return mag;
}

#else

double gamma(double*v,unsigned int deprecated){
  return v[0]*v[0] + v[1]*v[1];
}

double gamma(double*v1,double*v2,unsigned int deprecated){
  double a = (v1[0]-v2[0]);
  double b = (v1[1]-v2[1]);
  return a*a + b*b;
}

#endif
