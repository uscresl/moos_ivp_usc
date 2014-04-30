#include "USCutils.h"

void calcDxDyOperators2h(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y)
{
  double trig_angle;
  switch ( quadrant(lead_hdg) )
  {
    case 1:
      trig_angle = 90-lead_hdg;
      pos_x = false;
      break;
    case 2:
      trig_angle = lead_hdg-90;
      break;
    case 3:
      trig_angle = 270-lead_hdg;
      pos_y = false;
      break;
    case 4:
      trig_angle = lead_hdg-270;
      pos_x = false;
      pos_y = false;
      break;
    default:
      // shouldn't happen
      break;
  }
  delta_x = trig_dx(spacing, trig_angle);
  delta_y = trig_dy(spacing, trig_angle);
}
