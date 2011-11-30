ALGLIB_USED = alglib/alglibinternal.o alglib/ap.o alglib/specialfunctions.o alglib/interpolation.o alglib/alglibmisc.o alglib/optimization.o alglib/linalg.o alglib/dataanalysis.o alglib/integration.o alglib/solvers.o alglib/statistics.o

OBJS = gamma.o saturate.o Environment.o Obstacle.o MPN2D.o Integrator.o Unicycle.o Build.o 

MATLAB_PATH = matlabTests/
MATLAB_OBJS = bestPath.a getObPos.a negGradient.a samplePath.a
MATLAB_EXT = .mexglx

INCLUDE = -I ./ -I ../ -I ./alglib/
LIBPATH = -L ./

CFLAGS = -O3 -DDIM2

all: 

time_trial: time_trial.cpp $(OBJS) libalglib.so
	g++ $< $(OBJS) -o $@ -lconfig++ -lalglib $(LIBPATH) $(INCLUDE) $(CFLAGS)

test: test.cpp $(OBJS) vicon_multi.o libalglib.so
	g++ `pkg-config --cflags playerc++` $< $(OBJS) vicon_multi.o -o $@ `pkg-config --libs playerc++` -lViconDataStreamSDK_CPP -lconfig++ -lalglib $(LIBPATH) $(INCLUDE)

demo: demo.cpp $(OBJS) libalglib.so flat_control.o
	g++ `pkg-config --cflags playerc++` demo.cpp $(OBJS) flat_control.o -o demo `pkg-config --libs playerc++` -lconfig++ -lalglib $(INCLUDE) $(LIBPATH)


brownianSim: brownianSim.cpp $(OBJS) libalglib.so
	g++ `pkg-config --cflags playerc++` $< $(OBJS) -o $@ `pkg-config --libs playerc++` -lconfig++ -lalglib $(LIBPATH) $(INCLUDE)

MPN2D.o: MPN2D.cpp
	g++ MPN2D.cpp -Ialglib/ -c $(CFLAGS)

libalglib.so: $(ALGLIB_USED)
	g++ -shared -o $@ $^

alglib/%.o: alglib/%.cpp
	g++ $< -o $@ -c -fpic $(CFLAGS)

flat_control.o: flat_control.cpp
	g++ `pkg-config --cflags playerc++` $< -o $@ `pkg-config --libs playerc++` -c

%.o: %.cpp 
	g++ $< -o $@ -c $(CFLAGS)

clean:
	rm *.o *.so

install: libalglib.so
	cp $< /usr/lib/

uninstall: libalglib.so
	rm /usr/lib/$<