#include "USCutils.h"

size_t quadrant(double heading_in_degrees)
{
  // determine quadrant given heading (degrees 0-360)
  double by90 = heading_in_degrees / 90.0;
  double fractpart, intpart;
  fractpart = modf (by90, &intpart);
  return ((size_t)round(intpart) + 1);
}
