#ifndef __ROBOT_H
#define __ROBOT_H

class Robot {
 private:
  double x,y,yaw;
  double v,w;

 public:
  virtual void send() = 0; //Send the current v/w command
  virtual void refresh() = 0;//Update the data in the robot

  double getX(){return this->x;}
  double getY(){return this->y;}
  double getYaw(){return this->yaw;}
  
  void setV(double v){this->v = v;}
  void setW(double w){this->w = w;}

  double getV(){return this->v;} //For robots that measure their own speed
  double getW(){return this->w;}

 protected:
  void setX(double x){this->x = x;}
  void setY(double y){this->y = y;}
  void setYaw(double yaw){this->yaw = yaw;}
};

#endif
