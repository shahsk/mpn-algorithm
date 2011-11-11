#include "vicon_multi.hh"
//#include <libplayerc++/playerc++.h>
#include <math.h>

#define SLEEP 10
#define ANGULAR .25
#define ERROR .01


bool within_tol(float a,float b){
  return fabs(a-b) < ERROR;
}

float speed_controller(float desired,float real){
  float speed;
  speed = .50507048126+.08599699550479*log( fabs(desired-real) );
  cout << speed;
  if(desired > real)
    return speed;
  else
    return -speed;
}
/*
int main(){
  float pos[3],rpy[3],target,tmp;
  vicon_pos v("base");
  PlayerClient c(ROBOTIP,ROBOTPORT);
  Position2dProxy place(&c,0);

  place.SetSpeed(0,0);
  cout << "Press enter to start.";
  cin.get();

  for(;;){
    //get new information from the Vicon
    v.update();
  
    //get coordinates of the "base" segment of subject "nick"
    //from the latest frame.
    v.get_coord("nick",pos[0],pos[1],pos[2]);
    //cout << "Pos: " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
    v.get_euler_xyz("nick",rpy[0],rpy[1],rpy[2]);
    //cout << "Rpy: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;

    if( !within_tol(rpy[1],0)){
      tmp = rpy[1]; // to keep track of sign
      c.Peek(1000);
      place.SetSpeed(0,speed_controller(0,rpy[1]));
      
      while(!within_tol(rpy[1],0) && tmp*rpy[1] > 0){
	//get new information from the Vicon
	v.update();
	
	//get coordinates of the "base" segment of subject "nick"
	//from the latest frame.
	v.get_coord("nick",pos[0],pos[1],pos[2]);
	//cout << "Pos: " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
	v.get_euler_xyz("nick",rpy[0],rpy[1],rpy[2]);
	cout << "Rpy: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;
      }
      
    }
    c.Peek(1000);
    place.SetSpeed(0,0);


  }
  cout << "Done!\n";

  }*/

vicon_pos::vicon_pos(string segmentname){
  
  c = new Client;

  std::cout << "Connecting to Vicon...\n";
  while(!c->IsConnected().Connected){
    c->Connect(VICONIP);
    usleep(SLEEP);
  };
  c->EnableSegmentData();
  c->EnableMarkerData();
  c->EnableUnlabeledMarkerData();
  c->SetAxisMapping(ViconDataStreamSDK::CPP::Direction::Forward,
		    ViconDataStreamSDK::CPP::Direction::Left,
		    ViconDataStreamSDK::CPP::Direction::Up );
  c->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);

  c->GetFrame();

  num_subs = c->GetSubjectCount().SubjectCount;
  std::cout << "Found " << num_subs << " subjects.\n";

  subjects = new vector<string>;
  positions = new vector<point3d>;
  rpy = new vector<point3d>;
  positions->resize(num_subs);
  rpy->resize(num_subs);
  for(i =0; i<num_subs; i++){
    subjects->push_back(c->GetSubjectName(i).SubjectName);
  }
  
  segment = new string(segmentname);

  prev = c->GetFrameNumber().FrameNumber;
  curr = c->GetFrameNumber().FrameNumber;
  
}

//tries to connect to the created client, returns true for success
bool vicon_pos::connect(){
  c->Connect(VICONIP);
  return c->IsConnected().Connected;
}

//updates values measured by the vicon: refpt1, refpt2 and targets 
void vicon_pos::update(){
  c->GetFrame();
  while(c->GetFrameNumber().FrameNumber == prev){
    c->GetFrame();
    usleep(SLEEP);
  }
  prev = curr;
  curr = c->GetFrameNumber().FrameNumber;

  for(i=0; i<num_subs; i++){
    temp = c->GetSegmentGlobalTranslation((*subjects)[i],*segment);
    temp_rpy = c->GetSegmentGlobalRotationEulerXYZ((*subjects)[i],*segment);
    (*positions)[i].p[0] = temp.Translation[0]/1000.;
    (*positions)[i].p[1] = temp.Translation[1]/1000.;
    (*positions)[i].p[2] = temp.Translation[2]/1000.;

    (*rpy)[i].p[0] = temp_rpy.Rotation[0];
    (*rpy)[i].p[1] = temp_rpy.Rotation[1];
    (*rpy)[i].p[2] = temp_rpy.Rotation[2];

  }

}
//returns the given coordinate of the given target position relative to the robot
void vicon_pos::get_coord(unsigned int index,float& x,float& y,float& z){
  x = (*positions)[index].p[0];
  y = (*positions)[index].p[1];
  z = (*positions)[index].p[2];

  //std::cout << x << "," << y << "," << z << endl;
}

void vicon_pos::get_coord(string subjectname,float& x,float& y,float& z){
  for(i=0; i<num_subs; i++){
    if((*subjects)[i] == subjectname)
      break;
  }
  
  x = (*positions)[i].p[0];
  y = (*positions)[i].p[1];
  z = (*positions)[i].p[2];  

}

void vicon_pos::get_euler_xyz(unsigned int index,float& x,float& y,float& z){
  x = (*rpy)[index].p[0];
  y = (*rpy)[index].p[1];
  z = (*rpy)[index].p[2];
}
void vicon_pos::get_euler_xyz(string subjectname,float& x,float& y,float& z){
  for(i=0; i<num_subs; i++){
    if((*subjects)[i] == subjectname)
      break;
  }
  
  x = (*rpy)[i].p[0];
  y = (*rpy)[i].p[1];
  z = (*rpy)[i].p[2];  
}

point3d::point3d(){
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
}

point3d::point3d(float x,float y,float z){
  p[0] = x;
  p[1] = y;
  p[2] = z;
}

void vicon_pos::disconnect(){
  c->Disconnect();
}
