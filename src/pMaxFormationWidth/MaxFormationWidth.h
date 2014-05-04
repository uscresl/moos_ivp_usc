/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: MaxFormationWidth.cpp                                */
/*    DATE: Apr 24, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#ifndef PTEMPLATE_HEADER
#define PTEMPLATE_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

#include <boost/geometry.hpp>
#include <boost/geometry/io/wkt/read.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include "MBUtils.h"
#include "USCutils.h"

class MaxFormationWidth : public CMOOSApp
{
  public:
    MaxFormationWidth();
    ~MaxFormationWidth() {};

  protected: 
  // Standard MOOSApp functions to overload
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();

    // Registration, Configuration, Mail handling utils
    void registerVariables();
    bool handleMailMaxFormationWidthVarIn(std::string);

  private: 
    // Own functions
    void checkUpcomingLakeOutline(double const lead_x, double const lead_y, double const lead_hdg, double & max_form_width);

    // util funcs for WKT/Geometry
    void convertToWKT(std::string & str);
    typedef boost::geometry::model::d2::point_xy<double> b_point_2d;
    typedef boost::geometry::model::polygon<b_point_2d> b_polygon;
    void GeometryFromWKT(std::string const wkt, b_polygon & poly);

    // util funcs
    void publishToView(std::string const str);

    // Configuration variables
    double m_sensor_range;
    double m_sensor_width;

    // State variables
    bool debug;
    b_polygon m_lake_outline;
};

#endif 
