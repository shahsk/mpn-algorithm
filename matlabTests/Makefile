#If things fail to compile, first make sure that ou conpiled everything
#in the directory up
#If that already worked, then you are missing libconfig++.
#To install the included copy of libconfig:
# tar -zxvf libconfig-1.4.7.tar.gz 
# cd libconfig-1.4.7
# ./configure
# make
# sudo make install

all: configure.o nominalPath potentialField bestPath getObPos

nominalPath: configure.o
	matlab -nojvm -r "mex nominalPath.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 

potentialField: configure.o
	matlab -nojvm -r "mex potentialField.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 

negGradient: configure.o
	matlab -nojvm -r "mex negGradient.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 

samplePath: configure.o
	matlab -nojvm -r "mex samplePath.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 

bestPath: configure.o
	matlab -nojvm -r "mex bestPath.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 

getObPos: configure.o
	matlab -nojvm -r "mex getObPos.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 


#test: 
#	g++ test.cpp *.o ../*.o ../alglib/*.o -I../ -lconfig++

#setupEnv.o: setupEnv.cpp
#	g++ setupEnv.cpp -I../ -c

configure.o: configure.cpp
	g++ configure.cpp -I../ -c

clean:
	rm *.o