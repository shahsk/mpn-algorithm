#ifndef __GAMMA_H
#define __GAMMA_H

#include "datatypes.h"

mpn_float gamma(mpn_float*,unsigned int) __attribute__((pure));
mpn_float gamma(mpn_float*,mpn_float*,unsigned int) __attribute__((pure));

#endif