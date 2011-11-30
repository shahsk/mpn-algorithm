#ifndef __SATURATE_H
#define __SATURATE_H

#include "datatypes.h"

mpn_float sign(mpn_float num);
void saturate(mpn_float * val,mpn_float upper,mpn_float lower);

#endif
