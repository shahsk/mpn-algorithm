/*
 * Obstacle.h
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

class Obstacle {
private:
  unsigned int dim;
public:
  double * pos;//x,y(,z) position
  double radius;//size of the obstacle
  
  Obstacle(double * position,double r, unsigned int dim = 2);
  virtual ~Obstacle();
  
  double calculateBeta(double* q);//calculate the beta value of this obstacle at a given point
  double calculateDbeta(double* q,unsigned int d);//calculate the given partial of beta at a given point
};


#endif /* OBSTACLE_H_ */
