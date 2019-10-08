#include "commons.h"

int is_in(double x, double min, double max) {
  if(x < min) return 0;
  if(x > max) return 0;
  return 1;
}
