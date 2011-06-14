all: clean configure.o nominalPath potentialField bestPath

nominalPath: configure.o
	matlab -nojvm -r "mex nominalPath.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 

potentialField: configure.o
	matlab -nojvm -r "mex potentialField.cpp configure.o ../MPN2D.o ../Obstacle.o ../Environment.o ../alglib/ap.o ../alglib/alglibinternal.o ../alglib/specialfunctions.o -I../ -lconfig++; quit()" 

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