#include "commons.h"

int is_in(double x, double lower, double upper) {
  if(x < lower) return 0;
  if(x > upper) return 0;
  return 1;
}
