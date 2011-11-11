#If things fail to compile, first make sure that ou conpiled everything
#in the directory up
#If that already worked, then you are missing libconfig++.
#To install the included copy of libconfig:
# tar -zxvf libconfig-1.4.7.tar.gz 
# cd libconfig-1.4.7
# ./configure
# make
# sudo make install

INCLUDE = -I../ -I../alglib
LIBPATH = -L../
LIBS = -lconfig++ -lalglib

OBJS = ../gamma.o ../saturate.o ../Environment.o ../Obstacle.o ../MPN2D.o ../Integrator.o ../Unicycle.o ../Build.o 

PREFIX = matlab -nojvm -r "mex

SUFFIX = ; quit()" 

all: nominalPath potentialField bestPath getObPos

%: %.cpp
	$(PREFIX) $< $(OBJS) $(INCLUDE) $(LIBPATH) $(LIBS) $(SUFFIX) 