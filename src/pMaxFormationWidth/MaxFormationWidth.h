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

    // util funcs, TODO: move to own util library
    double deg2rad(double degrees);
    double rad2deg(double radians);
    double dx(double range, double trig_angle);
    double dy(double range, double trig_angle);
    size_t quadrant(double lead_heading);
    void calcDxDyOperatorsStd(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y);
    void calcDxDyOperators2h(double const spacing, double const lead_hdg, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y);
    double getDoubleFromNodeReport(std::string full_string, std::string key);
    std::string getStringFromNodeReport(std::string full_string, std::string key);
    void euclidDistance(double const x1, double const y1, double const x2, double const y2, double & euclid);

    // Configuration variables
    double m_time_horizon;
    std::string m_lead_vehicle;
    double m_ivd;

    // State variables
    size_t m_num_vehicles;
    bool debug;
    b_polygon m_lake_outline;
};

#endif 
