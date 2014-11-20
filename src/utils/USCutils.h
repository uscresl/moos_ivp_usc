/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: MaxFormationWidth.cpp                                */
/*    DATE: Apr 24, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#ifndef UTILS_HEADER
#define UTILS_HEADER

#include "MBUtils.h"
#include "math.h"
#include <stdlib.h>
#include <limits>

// angle utils
double deg2rad(double degrees);
double rad2deg(double radians);
size_t quadrant(double heading_in_degrees);

// trigonometry
double trig_dx(double const hypotenuse, double const trig_angle);
double trig_dy(double const hypotenuse, double const trig_angle);

// calculate euclidean distances
void euclidDistance(double const x1, double const y1, double const x2, double const y2, double & euclid);

// MOOS utils
double getDoubleFromNodeReport(std::string full_string, std::string key);
std::string getStringFromNodeReport(std::string full_string, std::string key);
double getDoubleFromCommaSeparatedString(std::string full_string, std::string key);

// CS599 calculations formations
void calcDxDyOperatorsStd(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y);
void calcDxDyOperators2h(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y);

#endif 
