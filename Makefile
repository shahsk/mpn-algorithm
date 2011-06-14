ALGLIB_USED = alglib/alglibinternal.o alglib/ap.o alglib/specialfunctions.o
OBJS = Environment.o Obstacle.o MPN2D.o

all: test

test: test.cpp $(OBJS) alglib
	g++ test.cpp $(OBJS) $(ALGLIB_USED) -o test
Environment.o: Environment.cpp
	g++ Environment.cpp -c
Obstacle.o: Obstacle.cpp
	g++ Obstacle.cpp -c
MPN2D.o: MPN2D.cpp
	g++ MPN2D.cpp -Ialglib/ -c
alglib: $(ALGLIB_USED)

clean:
	rm *.o alglib/*.o