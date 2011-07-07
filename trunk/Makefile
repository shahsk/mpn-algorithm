ALGLIB_USED = alglib/alglibinternal.o alglib/ap.o alglib/specialfunctions.o
OBJS = Environment.o Obstacle.o MPN2D.o

all: brownianSim test

test: test.cpp $(OBJS) alglib
	g++ `pkg-config --cflags playerc++` test.cpp $(OBJS) alglib/*.o matlabTests/configure.o -o test `pkg-config --libs playerc++` -lconfig++ -I./ -I./alglib

brownianSim: brownianSim.cpp $(OBJS) alglib
	g++ `pkg-config --cflags playerc++` brownianSim.cpp $(OBJS) $(ALGLIB_USED) -o brownianSim `pkg-config --libs playerc++`

Environment.o: Environment.cpp
	g++ Environment.cpp -c
Obstacle.o: Obstacle.cpp
	g++ Obstacle.cpp -c
MPN2D.o: MPN2D.cpp
	g++ MPN2D.cpp -Ialglib/ -c
alglib: $(ALGLIB_USED)

clean:
	rm *.o alglib/*.o