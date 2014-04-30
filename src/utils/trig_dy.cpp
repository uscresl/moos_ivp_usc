#include "USCutils.h"

double trig_dy(double const hypotenuse, double const trig_angle)
{
  return ( hypotenuse * cos( deg2rad(trig_angle) ) );
}
