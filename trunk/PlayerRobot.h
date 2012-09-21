#ifndef __PLAYERROBOT_H
#define __PLAYERROBOT_H

#include "Robot.h"
#include <libplayerc++/playerc++.h>

class PlayerRobot: public Robot{
private:
  PlayerCc::PlayerClient * sourceClient,* sinkClient;
  PlayerCc::Position2dProxy * sourcePos, * sinkPos;

public:
  PlayerRobot(PlayerCc::PlayerClient * sourceClient,
	      PlayerCc::Position2dProxy * sourcePos,
	      PlayerCc::PlayerClient * sinkClient,
	      PlayerCc::Position2dProxy * sinkPos);

  PlayerRobot(PlayerCc::PlayerClient * client,
	      PlayerCc::Position2dProxy * pos);

  void send(); //Send the current v/w command
  void refresh();//Update the data in the robot

};

#endif
