/*
 * Environment.h
 *
 *  Created on: Jun 7, 2011
 *      Author: nlacock
 */

#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

//Type identifier strings for building
#define ENVIRONMENT "environment"
#define DIPLOAR_ENVIRONMENT "dipolar_environment"

#include "Obstacle.h"
#include <libconfig.h++>
#include <vector>

/*
  Obstacles are optional, and are put in a list called obstacles. Each has a 
  radius and position.
  Config grammar example:

  environment:{
     potential_parameter = 3.0;
     radius = 10.0;
     destination = [0.0,0.0];
     obstacles = ({
       position = [1.0,1.0];
       radius = .2;
     },
     {
       position = [2.0,2.0];
       radius = .2;
     });
  };
*/
class Environment {//assumed to be centered at 0,0
 protected:
  double k;//tuning parameter for potential field

  unsigned int dim;
  
  std::vector<double> obstacleBetaValues;
  double envBeta;

 public:
  double radius;//size of the workspace
  double * goal;
  std::vector<Obstacle *> obstacles;

  Environment(double * destination,double k_in,double rad,unsigned int dim = 2);
  //Alternate constructor to use config files, pass in the environment group
  Environment(libconfig::Setting & group,unsigned int dim = 2);

  double calculateBeta(double * q);//return beta of the whole workspace, including obstacles. also refreshes internal beta values
  double calculateBeta0(double * q); //return just beta value of the workspace
  
  //puts the negated gradient at q in answer
  virtual void negatedGradient(double * q,double * answer);
  //returns the value of the potential field at a given point
  virtual double potentialField(double * q);
  
  virtual ~Environment();
};

/*
  Dipolar environment requires all of the parameters of environment, plus an
  epsilon and goal orientation. Name the group dipolar_environment

  NOTE: Dipolar environment is only defined for dimension 2 (I think)
  
  Config grammar example:

  dipolar_environment:{
     epsilon = .1;
     goal_orientation = 3.14;

     potential_parameter = 3.0;
     radius = 10.0;
     destination = [0.0,0.0];
     obstacles = ({
       position = [1.0,1.0];
       radius = .2;
     },
     {
       position = [2.0,2.0];
       radius = .2;
     });
  };


*/
class DipolarEnvironment: public Environment{
 private:
  double epsilon,goalOrientation,sinGoalOri,cosGoalOri;

 public:
  DipolarEnvironment(double * destination,double k_in,double rad,
		     double ep,double goalOri);
  DipolarEnvironment(libconfig::Setting & group);

  void negatedGradient(double * q,double * answer);
  double potentialField(double * q);

};


#endif /* ENVIRONMENT_H_ */
