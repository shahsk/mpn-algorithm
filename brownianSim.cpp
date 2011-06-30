#include "MPN2D.h"
#include "Environment.h"
#include "Obstacle.h"
#include <math.h>
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <iostream>
#include <time.h>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

#define ROBOTIP "localhost"
#define ROBOTPORT 6665


inline double norm(double x,double y){
  return sqrt(pow(x,2)+pow(y,2));
}

using namespace PlayerCc;

/*
  Simulates Shridhar's paper: a point robot moves through a workspace 
  (potentially with obstacles) and is distubed by a random normal velocity at
  every iteration. Stage can be launched in the stageSim directory, and 
  obstacles added by both entering their information here (commented out 
  example) or in simple.world (commented-out example). K is the control gain.
*/
int main( ){
  
  //Config
  double goal[2] = {0,0};
  double radius = 5.0;
  double epsilon = .1;
  double K = 1;

  Environment e(goal,1.0,radius);
  
  //Obstacle setup
  //double obPos1[2] = {-3.0,-1.0};
  //double obPos2[2] = {-2.0,-2.0};
  //e.obstacles.push_back(Obstacle(obPos1,1));
  //e.obstacles.push_back(Obstacle(obPos2,1));
  
  PlayerClient client(ROBOTIP,ROBOTPORT);
  Position2dProxy pos(&client,0);
  
  double position[2],negGradient[2],desired[2],angle,curr_time,
    prev_time,tmp;
  
  //Instantiate random number generator
  srand(time(NULL));
  boost::mt19937 seedX,seedY;	
  seedX.seed(rand());
  seedY.seed(rand());
  
  boost::normal_distribution<> dist(0, 1);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > XError(seedX,dist),  YError(seedY,dist);
  
  timeval now,prev;
  gettimeofday(&prev,NULL);
  prev_time = prev.tv_sec + (prev.tv_usec/1000000.0);
  
  do{
    
    //Update information
    client.Read();
    position[0] = pos.GetXPos();
    position[1] = pos.GetYPos();
    angle = pos.GetYaw();
    
    gettimeofday(&now,NULL);
    curr_time = now.tv_sec + (now.tv_usec/1000000.0);
    
    tmp = sqrt(curr_time-prev_time);
    prev_time = curr_time;
    //std::cout << tmp << std::endl;
    
    //Calculate the desired direction
    e.negatedGradient(position,negGradient);
    desired[0] = K*(negGradient[0]*cos(angle)- negGradient[1]*sin(angle));
    desired[1] = K*(negGradient[0]*sin(angle)+ negGradient[1]*cos(angle));
    
    pos.SetSpeed(-(desired[0]+XError()*tmp),-(desired[1]+YError()*tmp),0);
    
    //std::cout << norm(position[0],position[1]) << std::endl;

  }while( norm(position[0],position[1]) > epsilon && 
	  norm(position[0],position[1]) < radius );
  
}
