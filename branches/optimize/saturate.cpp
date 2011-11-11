#include "saturate.h"
#include "math.h"
void saturate(double * val,double upper,double lower){
  if(fabs(*val) > upper && upper > 0){
    *val = sign(*val)*upper;
  }
  else if(*val < lower && lower > 0){
    *val = sign(*val)*lower;
  }

}

double sign(double num){
  if(num > 0)
    return 1.0;
  else if(num < 0)
    return -1.0;
  else
    return 0.0;
}
