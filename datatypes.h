/*
  This file contains macro definitions for datatypes used in this 
  implementation.
*/

/*
  Change the precision of the implementation. Default is double precision.
  ex usage:
  gcc Obstacle.cpp -o -DSINGLE_PRECISION
*/

#ifndef __DATATYPES_H
#define __DATATYPES_H

#ifdef SINGLE_PRECISION
#define mpn_float float
#endif

#ifdef QUADRUPLE_PRECISION
#define mpn_float long double
#endif

#ifndef mpn_float
#define mpn_float double
#endif

#endif /*__DATATYPES_H*/
