#include "SelectFormation.h"

double SelectFormation::deg2rad(double degrees)
{
  return ( degrees * (M_PI/180) );
}

double SelectFormation::rad2deg(double radians)
{
  std::cout << "bla\n";
  return ( radians * (180 / M_PI) );
}

double SelectFormation::dx(double range, double trig_angle)
{
  return ( range * sin( deg2rad(trig_angle) ) );
}

double SelectFormation::dy(double range, double trig_angle)
{
  return ( range * cos( deg2rad(trig_angle) ) );
}

size_t SelectFormation::quadrant(double lead_heading)
{
  // determine quadrant given heading (degrees 0-360)
  double by90 = lead_heading / 90.0;
  double fractpart, intpart;
  fractpart = modf (by90, &intpart);
  size_t tmp = ((size_t)round(intpart) + 1);
  std::cout << "quadrant: " << tmp << std::endl;
  return tmp;
}
