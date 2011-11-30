#include "datatypes.h"
#include "saturate.h"
#include "math.h"
void saturate(mpn_float * val,mpn_float upper,mpn_float lower){
  if(fabs(*val) > upper && upper > 0){
    *val = sign(*val)*upper;
  }
  else if(*val < lower && lower > 0){
    *val = sign(*val)*lower;
  }

}

mpn_float sign(mpn_float num){
  if(num > 0)
    return 1.0;
  else if(num < 0)
    return -1.0;
  else
    return 0.0;
}
