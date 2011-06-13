#ifndef CONFIGURE_H_
#define CONFIGURE_H_

#include <string>
#include "Environment.h"
#include "Obstacle.h"
#include "MPN2D.h"

void configure(Environment * & e,MPNParams * & mp);
void configure(char file[],Environment * & e,MPNParams * & mp);

#endif
