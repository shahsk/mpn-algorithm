ALGLIB_USED = alglib/alglibinternal.o alglib/ap.o alglib/specialfunctions.o alglib/interpolation.o alglib/alglibmisc.o alglib/optimization.o alglib/linalg.o alglib/dataanalysis.o alglib/integration.o alglib/solvers.o alglib/statistics.o
OBJS = gamma.o Environment.o Obstacle.o MPN2D.o Integrator.o Unicycle.o Build.o vicon_multi.o
all: brownianSim test

test: test.cpp $(OBJS) alglib
	g++ `pkg-config --cflags playerc++` test.cpp $(OBJS) $(ALGLIB_USED) -o test `pkg-config --libs playerc++` -lViconDataStreamSDK_CPP -lconfig++ -I./ -I./alglib

brownianSim: brownianSim.cpp Environment.o Obstacle.o gamma.o alglib
	g++ `pkg-config --cflags playerc++` brownianSim.cpp gamma.o Environment.o Obstacle.o $(ALGLIB_USED) -o brownianSim `pkg-config --libs playerc++` -lconfig++

MPN2D.o: MPN2D.cpp
	g++ MPN2D.cpp -Ialglib/ -c
alglib: $(ALGLIB_USED)

clean:
	rm *.o #alglib/*.o