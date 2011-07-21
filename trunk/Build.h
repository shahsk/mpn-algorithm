#ifndef __BUILD_H
#define __BUILD_H

#include <libconfig.h++>
#include "MPN2D.h"
#include "Environment.h"
#include "Integrator.h"
#include "Unicycle.h"

/*
  Builds objects polymorphically based on the given cfg file or object. 
  Objects should have their string identifiers defined in their header file.
  If an object is not built, the pointer reference is set to NULL

*/

//Use these if each piece is in a different file, or if you only need 1 piece
void buildEnvironment(const char * filename,Environment * & env,int wsdim);
void buildIntegrator(const char * filename,Integrator * & intg,int wsdim,
		     double stepTime);
void buildMPNParams(const char * filename,MPNParams * & mp);

//These are more efficient if all of your information is in the same file
void buildEnvironment(libconfig::Config * cfg,Environment * & env,int wsdim);
void buildIntegrator(libconfig::Config * cfg,Integrator * & intg,int wsdim,
		     double stepTime);
void buildMPNParams(libconfig::Config * cfg,MPNParams * & mp); 

//If everything is in the same file, this is the easiest
void buildAll(const char * filename,Environment * & env, Integrator * & intg,
	      MPNParams * & mp,double stepTime);

#endif
