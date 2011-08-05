#include "alglib/interpolation.h"
#include "MPN2D.h"
#include "Integrator.h"
#include "Build.h"
#include "Environment.h"
#include "Obstacle.h"
#include "gamma.h"
#include "saturate.h"
#include <math.h>
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <iostream>
#include <time.h>
#include <libconfig.h++>
#include <string>
#include <fstream>

#include "vicon_multi.hh"
#include <boost/thread.hpp>


#define ROBOTIP "192.168.1.105"
#define ROBOTPORT 6666

#define PRECISION .01
#define DEGREE 10

//Controller proportions
#define KPX 1
#define KDX 2
#define KPY 1
#define KDY 2

//-1 means unconstrained
#define VMAX .3
#define VMIN .1
#define OMEGAMAX .5
#define OMEGAMIN -1

#define POLYTIME params->controlHorizon;

#define CMD_RATE .3

using namespace PlayerCc;


double saturate(double val, double min, double max){
  if(fabs(val) > min && fabs(val) < max)
    return val;

  if(fabs(val) <= min){
    if(val > 0)
      return min;
    else
      return -min;
  }
  
  if(fabs(val) >= max){
    if(val > 0)
      return max;
    else
      return -max;
  }

}

double satv(double v){return saturate(v,VMIN,VMAX); }
double satw(double w){return saturate(w,OMEGAMIN,OMEGAMAX); }
/*
//Emulate the robot's minimum velocity for simulation
double satv(double v){
  if(fabs(v) < VMIN)
    return 0;
  else
    return saturate(v,VMIN,VMAX);
}

double satw(double v){
  if(fabs(v) < OMEGAMIN)
    return 0;
  else
    return saturate(v,OMEGAMIN,OMEGAMAX);
}


int sign(double a){
  if(a>0)
    return 1;
  else
    return -1;
}
*/

//Builds 2 polynomials x(t) and y(t) on the given x,y trajectory. t ranges from 0 to 1
void buildPolynoms(double ** path,double ** pathDeriv,int steps,double range,
		  alglib::barycentricinterpolant * xpoly,
		  alglib::barycentricinterpolant * ypoly){

  //Copy the path into arrays
  alglib::real_1d_array x,y,t,weights,tc,xc,yc;
  x.setlength(steps+1);
  y.setlength(steps+1);
  t.setlength(steps+1);
  weights.setlength(steps+1);

  double dt = range/static_cast<double>(steps);
  for(int i(0); i<steps+1; i++){
    t[i] = (i)*dt;
    x[i] = path[i][0];
    y[i] = path[i][1];
    weights[i] = 1;

    //std::cout << x[i] << "," << y[i] << "\n";
  }


  //std::cout << dt << std::endl;
  //std::cout << path[steps][0] << "," << path[steps][1] << std::endl;
  
  int info;
  //alglib::polynomialfitreport rep;
  alglib::barycentricfitreport rep;
  /*
  polynomialfit(t,x,steps+1,DEGREE,info,*xpoly,rep);
  polynomialfit(t,y,steps+1,DEGREE,info,*ypoly,rep);
  */

  //put constraints on the endpoint
  tc.setlength(2);
  xc.setlength(2);
  yc.setlength(2);

  tc[0] = range;
  tc[1] = 0;
  //tc[2] = range;

  xc[0] = path[steps][0];
  xc[1] = path[0][0];
  //xc[2] = (path[steps+1][0]-path[steps][0])/dt;

  yc[0] = path[steps][1];
  yc[1] = path[0][1];
  //yc[2] = (path[steps+1][1]-path[steps][1])/dt;

  alglib::integer_1d_array type("[0,0,1]");

  /*
alglib::barycentricfitfloaterhormannwc(
    real_1d_array x,
    real_1d_array y,
    real_1d_array w,
    ae_int_t n,
    real_1d_array xc,
    real_1d_array yc,
    integer_1d_array dc,
    ae_int_t k,
    ae_int_t m,
    ae_int_t& info,
    barycentricinterpolant& b,
    barycentricfitreport& rep);*/

  //polynomialfitwc(t,x,weights,tc,xc,type,DEGREE,info,*xpoly,rep);
  barycentricfitfloaterhormannwc(t,x,weights,t.length(),tc,xc,type,tc.length(),DEGREE,info,*xpoly,rep);
  //std::cout << info << std::endl;
  //polynomialfitwc(t,y,weights,tc,yc,type,DEGREE,info,*ypoly,rep);
  barycentricfitfloaterhormannwc(t,y,weights,t.length(),tc,yc,type,tc.length(),DEGREE,info,*ypoly,rep);
  //std::cout << info << std::endl;

  /*
  alglib::real_1d_array coefs;
  polynomialbar2pow(*xpoly,coefs);
  std::cout << "xp = [";
  for(int i(coefs.length()-1); i>-1; i--)
    std::cout << coefs[i] << ",";
  std::cout << "];\n";

  polynomialbar2pow(*ypoly,coefs);
  std::cout << "yp = [";
  for(int i(coefs.length()-1); i>-1; i--)
    std::cout << coefs[i] << ",";
  std::cout << "];\n";
  */
  
}

struct trajController{
  PlayerClient * client,* me;
  Position2dProxy * pos,*drive;

  double endGoal[2];

  double goal[2],pose[3],currVel[2],tol,desiredV[2],desiredX[2],desiredA[2],u[2],v,omega,acc,dt;
  int steps;

  timeval start,now;
  double startTime,currTime,prevTime;

  double prevVel,prevOmega;
  trajController(double * endGoal,double tol){
    
    this->endGoal[0] = endGoal[0];
    this->endGoal[1] = endGoal[1];

    client = new PlayerClient(ROBOTIP,ROBOTPORT);
    //me = new PlayerClient("localhost",6665);
    me = client;
    drive = new Position2dProxy(client,0);
    //pos = new Position2dProxy(me,0);
    pos = drive;

    this->tol = tol;

    me->Read();
    pose[0] = pos->GetXPos();
    pose[1] = pos->GetYPos();
    pose[2] = pos->GetYaw();

  }

  void operator()(alglib::barycentricinterpolant * xpath,
		  alglib::barycentricinterpolant * ypath,
		  double time,bool correct){

    //std::cout << "path = [";
    goal[0] = barycentriccalc(*xpath,time);
    goal[1] = barycentriccalc(*ypath,time);

    steps = ceil(time/PRECISION);
    dt = time/steps;

    gettimeofday(&start,NULL);
    startTime = start.tv_sec + start.tv_usec/1000000.;
    currTime = startTime;
    prevTime = startTime;

    bool first = true;
    while(currTime - startTime < time && gamma(pose,goal,2) > tol*tol){
      dt = currTime - prevTime;
      //dt = CMD_RATE;

      barycentricdiff2(*xpath,currTime-startTime,desiredX[0],desiredV[0],desiredA[0]);
      barycentricdiff2(*ypath,currTime-startTime,desiredX[1],desiredV[1],desiredA[1]);
      
      std::cout << desiredX[0] << "," << desiredX[1] << "," ;

      //std::cout << "desired x: " << desiredX[0] << " desired y: " << desiredX[1] << std::endl;
    
      me->Read();
      pose[0] = pos->GetXPos();
      pose[1] = pos->GetYPos();
      pose[2] = pos->GetYaw();

      std::cout << pose[0] << "," << pose[1] << "\n";

      //if(pose[2] <= 0)
      //pose[2] += 2*M_PI;
      //std::cout << pose[2] << std::endl;

      if(gamma(pose,endGoal,2) < tol*tol){
	drive->SetSpeed(0,0);
	break;
      }

      if(correct && first){
	//std::cout << "HEREHEREHEREHERE\n";
	currVel[0] = desiredV[0];
	currVel[1] = desiredV[1];
      }
      else{
	currVel[0] = pos->GetXSpeed()*cos(pose[2]);
	currVel[1] = pos->GetXSpeed()*sin(pose[2]);
	//currVel[0] = prevVel*cos(pose[2])*((rand()-rand())
	//1				   /static_cast<double>(RAND_MAX));
	//currVel[1] = prevVel*sin(pose[2])*((rand()-rand())
	//				   /static_cast<double>(RAND_MAX));

      }
      
      //std::cout << "currvel: " <<currVel[0] << "," << currVel[1] << std::endl;

      u[0] = desiredA[0] + KPX*(desiredX[0] - pose[0] ) + KDX*(desiredV[0] - currVel[0]);
      u[1] = desiredA[1] + KPX*(desiredX[1] - pose[1] ) + KDX*(desiredV[1] - currVel[1]);

      //std::cout << "u: " <<u[0] << "," << u[1] << std::endl;

      acc = u[0]*cos(pose[2]) + u[1]*sin(pose[2]);
      v = sqrt(gamma(currVel,2)) + acc*dt;
      omega = (u[1]*cos(pose[2]) - u[0]*sin(pose[2]))/(v);

      //std::cout << currVel[0] << "," << currVel[1] << std::endl;
      //std::cout << sin(pose[2]) << std::endl;


      //v = satv(v);
      saturate(&v,VMAX,VMIN);
      v = v > 0 ? v : 0; //Positive velocities only, thank you very much.
      //omega = satw(omega);
      saturate(&omega,OMEGAMAX,OMEGAMIN);
    
      drive->SetSpeed(v,omega);

      //std::cout << "vel: " << v << " omega: " << omega << std::endl;    

      prevTime = currTime;
      do{
	usleep(200);
	gettimeofday(&now,NULL);
	currTime = now.tv_sec + now.tv_usec/1000000.;
      }while(currTime-prevTime < CMD_RATE);

      prevVel = v;
      prevOmega = omega;
      first = false;
      //std::cout << currTime-prevTime << std::endl;
    }
    

    //std::cout << "DONE!\n";
    //std::cout << "Time used: " << currTime - startTime << std::endl;
    //std::cout << "];\n";
  }

};

int main(){
  vicon_pos tmpvicon("base");
  tmpvicon.update();

  //Setup
  char filename[] = "lab.cfg";
  Environment * env;
  MPNParams * params;
  Integrator * intgr;

  libconfig::Config c;
  c.readFile(filename);

  buildEnvironment(&c,env,2);
  buildMPNParams(&c,params);

  int j = 0;
  float temparray[3];
  for(unsigned int i(0); i<tmpvicon.subjects->size() && j<env->obstacles.size(); i++){
    if((*tmpvicon.subjects)[i].find("Obstacle") != std::string::npos){
      env->obstacles[j]->radius = .18;
      tmpvicon.get_coord(i,temparray[0],temparray[1],temparray[2]);
      env->obstacles[j]->pos[0] = temparray[0];
      env->obstacles[j]->pos[1] = temparray[1];
      std::cout << "obstacle at: " << env->obstacles[j]->pos[0] << "," << env->obstacles[j]->pos[1] << std::endl;
      j++;
    }
      
  }

  tmpvicon.disconnect();

  //std::cout << "goal: " << env->goal[0] << "," << env->goal[1] << std::endl;

  trajController robot(env->goal,params->tolerance);

  int steps,CHI,nextSteps,nextCHI; //CHI = Control Horizon Index
  double start[2],dt = PRECISION;//,time=params->controlHorizon;
  double time = POLYTIME;
  double ** bestPath,**bestControl,**nextPath,**nextControl;

  robot.me->Read();
  start[0] = robot.pos->GetXPos();
  start[1] = robot.pos->GetYPos();

  intgr = new Unicycle(dt,robot.pos->GetYaw(),VMAX,OMEGAMAX,VMIN,-1);
  //intgr = new Unicycle(dt,robot.pos->GetYaw());
  //intgr = new Integrator(dt,2);
  

  alglib::barycentricinterpolant *currXPoly,*currYPoly,*nextXPoly,*nextYPoly,*tmp;
  currXPoly = new alglib::barycentricinterpolant();
  currYPoly = new alglib::barycentricinterpolant();
  nextXPoly = new alglib::barycentricinterpolant();
  nextYPoly = new alglib::barycentricinterpolant();
  
  //Generate an initial path
  //std::cout << "Before: " << dynamic_cast<Unicycle *>(intgr)->currTheta << std::endl;
  bool done = generateBestPath(env,params,intgr,bestPath,bestControl,
			       steps,CHI,start);
  buildPolynoms(bestPath,bestControl,CHI,time,currXPoly,currYPoly);

  //std::cout << "displacement: " << sqrt(gamma(bestPath[0],bestPath[CHI],2)) << std::endl;

  bool dummy = true;
  double realPos[2] = {start[0],start[1]};
  while(!done){

    start[0] = bestPath[CHI][0];
    start[1] = bestPath[CHI][1];

    //start[0] = barycentriccalc(*currXPoly,time);//bestPath[CHI][0];
    //start[1] = barycentriccalc(*currYPoly,time);//bestPath[CHI][1];

    //std::cout << start[0] << "," << start[1] << std::endl;
    
    //Start driving along the path
    boost::thread driveThread(robot,currXPoly,currYPoly,time,dummy);

    //Compute the next path
    //std::cout << "Before: " << dynamic_cast<Unicycle *>(intgr)->currTheta << std::endl;
    done = generateBestPath(env,params,intgr,nextPath,nextControl,nextSteps,nextCHI,start);

    //std::cout << "After: " << dynamic_cast<Unicycle *>(intgr)->currTheta << std::endl;
    //std::cout << "start theta: " << dynamic_cast<Unicycle *>(intgr)->startTheta << "curr theta: " << dynamic_cast<Unicycle *>(intgr)->currTheta << std::endl;
    buildPolynoms(nextPath,nextControl,nextCHI,time,nextXPoly,nextYPoly);

    //std::cout << "nominal final pos: " << nextPath[nextCHI][0] << "," << nextPath[nextCHI][1] << std::endl;
    //std::cout << "displacement: " << sqrt(gamma(nextPath[0],nextPath[nextCHI],2)) << std::endl;

    //Set up for next iteration
    cleanupPoints(bestPath,steps);
    cleanupPoints(bestControl,steps);
    bestPath = nextPath;
    bestControl = nextControl;
    steps = nextSteps;
    CHI = nextCHI;

    //pointer swap
    tmp = currXPoly;
    currXPoly = nextXPoly;
    nextXPoly = tmp;

    tmp = currYPoly;
    currYPoly = nextYPoly;
    nextYPoly = tmp;
    
    params->currentTime += params->controlHorizon;
    if(params->currentTime > params->predictionHorizon)
      params->currentTime = 0;

    //Wait for robot to finish
    driveThread.join();
    realPos[0] = robot.pos->GetXPos();
    realPos[1] = robot.pos->GetYPos();

    dummy = false;
  }  

  //Drive the final stretch
  boost::thread driveThread(robot,currXPoly,currYPoly,time,dummy);
  driveThread.join();
  robot.drive->SetSpeed(0,0);
  sleep(3);
}
