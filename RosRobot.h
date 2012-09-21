#ifndef __ROSROBOT_H
#define __ROSROBOT_H

#include "Robot.h"

class RosRobot : public Robot{
 private:
  //Ros::publisher
  //Ros::subscriber
  void send(); //Send the current v/w command
  void refresh();//Update the data in the robot

}

#endif
