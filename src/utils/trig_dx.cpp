#include "USCutils.h"

double trig_dx(double const hypotenuse, double const trig_angle)
{
  return ( hypotenuse * sin( deg2rad(trig_angle) ) );
}
