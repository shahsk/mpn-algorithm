all: nominalPath.mexglx potentialField.mexglx

clean:
	rm *.o *.mexglx

nominalPath.mexglx: setupEnv.o nominalPath.cpp
	mex nominalPath.cpp setupEnv.o ../Debug/*.o ../Debug/src/*.o -I../

potentialField.mexglx: setupEnv.o potentialField.cpp
	mex potentialField.cpp setupEnv.o ../Debug/*.o ../Debug/src/*.o -I../

setupEnv.o: setupEnv.cpp
	g++ setupEnv.cpp -I../ -c