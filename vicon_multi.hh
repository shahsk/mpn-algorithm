#ifndef VICON_MULTI_HH
#define VICON_MULTI_HH

#include "Client.h"
#include <vector>
#include <iostream>
#include "stdlib.h"

//Constants
#define VICONIP  "192.168.1.110:801"

using namespace ViconDataStreamSDK::CPP;
using namespace std;

struct point3d{
  point3d();
  point3d(float,float,float);
  float p[3];
};

/*
  Wrapper class for the vicon system.
  Can handle any number of robots as long as the share a common segment name.
  Right now, it only maintains translation data of the segments.
*/
class vicon_pos{
  //vicon client class
  Client* c;


  //positions of subject's segment
  vector<point3d>* positions;

  //roll, pitch, yaw of segments
  vector<point3d>* rpy;
  
  //name of the segment to use for tracking on each subject, should be universal
  std::string* segment;
  
  unsigned int num_subs;

  //temporaries
  unsigned int i,curr,prev;
  Output_GetSegmentGlobalTranslation temp;
  Output_GetSegmentGlobalRotationEulerXYZ temp_rpy;

  bool connect();

public:
  //names of detected subjects
  vector<string>* subjects;


  //segment name MUST be common to all robots (ex. "base")
  vicon_pos(string segmentname);

  /*
    Puts the global coordinates of given subject in the given floats
    You can access subjects either by name or by index (the index is the order
    in which they were foud, USUALLY the reverse of the order they are listed in
    the Vicon Nexus 3D interface)

    Does NOT call update for you.
   */
  void get_coord(unsigned int,float&,float&,float&);
  void get_coord(string,float&,float&,float&);

  void get_euler_xyz(unsigned int,float&,float&,float&);
  void get_euler_xyz(string,float&,float&,float&);

  //gets a new frame and updates everything, should call once per cycle
  void update();

  void disconnect();
};

#endif
