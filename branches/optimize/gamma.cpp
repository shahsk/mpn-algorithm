#include "gamma.h"
#include <math.h>

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
