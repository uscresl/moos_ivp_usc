#include "USCutils.h"

void calcDxDyOperatorsStd(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y)
{
  double trig_angle;
  // calculate the angle for trig, given AUV heading
  switch ( quadrant(lead_hdg) )
  {
    case 1:
      trig_angle = lead_hdg;
      pos_x = false;
      pos_y = false;
      break;
    case 2:
      trig_angle = 180-lead_hdg;
      pos_x = false;
      break;
    case 3:
      trig_angle = lead_hdg-180;
      break;
    case 4:
      trig_angle = 360-lead_hdg;
      pos_y = false;
      break;
    default:
      // shouldn't happen
      break;
  }
  // calculate x/y displacement from trig_angle and follow range
  // trig_angle connected edge y, opposite edge x, given above calculations
  // cos trig_angle = dy / range
  // sin trig_angle = dx / range
  delta_x = trig_dx(spacing, trig_angle);
  delta_y = trig_dy(spacing, trig_angle);
}
