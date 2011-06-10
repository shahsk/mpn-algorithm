#TODO: learn how to do this the right way
testDerivMat.mexglx: testDerivMat.cpp setupEnv.o
	mex testDerivMat.cpp setupEnv.o ../Debug/*.o ../Debug/src/*.o -I../
testPotMat.mexglx: testPotMat.cpp setupEnv.o
	mex testPotMat.cpp setupEnv.o ../Debug/*.o ../Debug/src/*.o -I../
testPathMat.mexglx: testPathMat.cpp setupEnv.o
	mex testPathMat.cpp setupEnv.o ../Debug/*.o ../Debug/src/*.o -I../
setupEnv.o: setupEnv.cpp
	g++ setupEnv.cpp 