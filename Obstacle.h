/*
 * Obstacle.h
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "datatypes.h"

class Obstacle {
private:
  unsigned int dim;
public:
  mpn_float * pos;//x,y(,z) position
  mpn_float radius;//size of the obstacle
  
  Obstacle(mpn_float * position,mpn_float r, unsigned int dim = 2);
  virtual ~Obstacle();
  
  mpn_float calculateBeta(mpn_float* q);//calculate the beta value of this obstacle at a given point
  mpn_float calculateDbeta(mpn_float* q,unsigned int d);//calculate the given partial of beta at a given point
};


#endif /* OBSTACLE_H_ */
