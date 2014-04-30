#include "USCutils.h"

void euclidDistance(double const x1, double const y1, double const x2, double const y2, double & euclid)
{
  double dx = x1 - x2;
  double dy = y1 - y2;
  euclid = sqrt( (dx*dx) + (dy*dy) );
}
