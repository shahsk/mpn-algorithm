#include "Robot.h"
#include "PlayerRobot.h"
#include <libplayerc++/playerc++.h>

PlayerRobot::PlayerRobot(PlayerCc::PlayerClient * sourceClient,
			 PlayerCc::Position2dProxy * sourcePos,
			 PlayerCc::PlayerClient * sinkClient,
			 PlayerCc::Position2dProxy * sinkPos){
  this->sourceClient = sourceClient;
  this->sourcePos = sourcePos;

  this->sinkClient = sinkClient;
  this->sinkPos = sinkPos;
}

PlayerRobot::PlayerRobot(PlayerCc::PlayerClient * client,
			 PlayerCc::Position2dProxy * pos){
  this->sourceClient = this->sinkClient = client;
  this->sourcePos = this->sinkPos = pos;
}

void PlayerRobot::send(){
  this->sinkPos->SetSpeed(this->getV(),this->getW());
}
void PlayerRobot::refresh(){
  this->sourceClient->Read();
  this->setX( this->sourcePos->GetXPos() );
  this->setY( this->sourcePos->GetYPos() );
  this->setYaw( this->sourcePos->GetYaw() );

  //Expected behavior is that we get these measurements if they exist and 
  //get the last command if they don't, need to check
  this->setV( this->sourcePos->GetXSpeed() ); 
  this->setW( this->sourcePos->GetYawSpeed() );
}
