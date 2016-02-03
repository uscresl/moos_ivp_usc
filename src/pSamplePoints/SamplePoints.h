/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SamplePoints.h                                       */
/*    DATE: Jan 25, 2016                                         */
/*                                                               */
/*****************************************************************/

#ifndef PSAMPLEPOINTS_HEADER
#define PSAMPLEPOINTS_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

class SamplePoints : public CMOOSApp
{
  public:
    SamplePoints();
    ~SamplePoints() {};

  protected:
    // Standard MOOSApp functions to overload
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();

    // Registration, Configuration, Mail handling utils
    void registerVariables();

  private:
    // Own functions
    void calculateGridPoints(double ctr_x, double ctr_y, double width, double height, double lane_width);
    void initGeodesy();
    void publishGridPoints(std::vector<std::pair<double, double> > grid_vector);
    void publishGridSpecs(double width, double height, double lane_width);

    // Configuration variables
    std::string m_output_var_sample_points;
    std::string m_output_var_specs;

    // State variables
    bool m_got_aabbcc;

    // Class variables
    CMOOSGeodesy m_geodesy;

};

#endif 
