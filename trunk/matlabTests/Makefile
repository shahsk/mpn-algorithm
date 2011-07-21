#If things fail to compile, first make sure that ou conpiled everything
#in the directory up
#If that already worked, then you are missing libconfig++.
#To install the included copy of libconfig:
# tar -zxvf libconfig-1.4.7.tar.gz 
# cd libconfig-1.4.7
# ./configure
# make
# sudo make install

all: nominalPath potentialField bestPath getObPos

nominalPath: nominalPath.cpp
	matlab -nojvm -r "mex nominalPath.cpp ../*.o ../alglib/*.o -I../ -I../alglib -lconfig++; quit()" 

potentialField: potentialField.cpp
	matlab -nojvm -r "mex potentialField.cpp ../*.o ../alglib/*.o -I../ -lconfig++; quit()" 

negGradient: negGradient.cpp
	matlab -nojvm -r "mex negGradient.cpp ../*.o ../alglib/*.o -I../ -lconfig++; quit()" 

samplePath: samplePath.cpp
	matlab -nojvm -r "mex samplePath.cpp ../*.o ../alglib/*.o -I../ -lconfig++; quit()" 

bestPath: bestPath.cpp
	matlab -nojvm -r "mex bestPath.cpp ../*.o ../alglib/*.o -I../ -lconfig++; quit()" 

getObPos: getObPos.cpp
	matlab -nojvm -r "mex getObPos.cpp ../*.o ../alglib/*.o -I../ -lconfig++; quit()" 

clean:
	rm *.o