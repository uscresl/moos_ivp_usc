#include "MaxFormationWidth.h"

double MaxFormationWidth::deg2rad(double degrees)
{
  return ( degrees * (M_PI/180) );
}

double MaxFormationWidth::rad2deg(double radians)
{
  std::cout << "bla\n";
  return ( radians * (180 / M_PI) );
}

double MaxFormationWidth::dx(double range, double trig_angle)
{
  return ( range * sin( deg2rad(trig_angle) ) );
}

double MaxFormationWidth::dy(double range, double trig_angle)
{
  return ( range * cos( deg2rad(trig_angle) ) );
}

size_t MaxFormationWidth::quadrant(double lead_heading)
{
  // determine quadrant given heading (degrees 0-360)
  double by90 = lead_heading / 90.0;
  double fractpart, intpart;
  fractpart = modf (by90, &intpart);
  size_t tmp = ((size_t)round(intpart) + 1);
//  std::cout << "quadrant: " << tmp << std::endl;
  return tmp;
}

void MaxFormationWidth::calcDxDyOperatorsStd(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y)
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
  delta_x = dx(spacing, trig_angle);
  delta_y = dy(spacing, trig_angle);
}

void MaxFormationWidth::calcDxDyOperators2h(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y)
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
  delta_x = dx(spacing, trig_angle);
  delta_y = dy(spacing, trig_angle);
}

//---------------------------------------------------------
// Procedure: getDoubleFromNodeReport
//            retrieve any double value from node_report by name
//
double MaxFormationWidth::getDoubleFromNodeReport(std::string full_string, std::string key)
{
  std::string output = getStringFromNodeReport(full_string, key);
  return atof(output.c_str());
}

//---------------------------------------------------------
// Procedure: getStringFromNodeReport
//            retrieve any string value from node_report by name
//
std::string MaxFormationWidth::getStringFromNodeReport(std::string full_string, std::string key)
{
  // example: NAME=anton,X=2676.17,Y=1908.45,SPD=1.48,HDG=316.19,DEP=0,
  //   LAT=34.26380127,LON=-117.17504934,TYPE=SHIP,GROUP=survey,MODE=DRIVE,
  //   ALLSTOP=clear,index=57,YAW=316.19,TIME=1398119728.44,LENGTH=8

  // handle comma-separated string
  std::string output;
  bool valid_msg = true;
  std::vector<std::string> svector = parseString(full_string, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    std::string param = biteStringX(svector[i], '=');
    std::string value = svector[i];
    if(param == key)
      output = value;
    else
      valid_msg = false;
  }

  if( output == "" )
    valid_msg = false;

//  if(!valid_msg)
//    std::cout << GetAppName() << " :: Unhandled NODE_REPORT: " << full_string << std::endl;

  return output;
}

void MaxFormationWidth::euclidDistance(double const x1, double const y1, double const x2, double const y2, double & euclid)
{
  double dx = x1 - x2;
  double dy = y1 - y2;
  euclid = sqrt( (dx*dx) + (dy*dy) );
}
