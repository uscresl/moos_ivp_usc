/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SelectFormation.h                                    */
/*    DATE: Apr 21, 2014                                         */
/*                                                               */
/*    This process will choose what formation should be taken,   */
/*    based on the allowed width/height and number of vehicles   */
/*    And it will output a comma-separated list of positions in  */
/*    the formation.                                             */
/*                                                               */
/*    Nb. this is not an AppCastMOOSApp                          */
/*                                                               */
/*****************************************************************/

#ifndef PSELECT_FORMATION_HEADER
#define PSELECT_FORMATION_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

#include "math.h"

class SelectFormation : public CMOOSApp
{
  public:
    SelectFormation();
    ~SelectFormation() {};

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
    void updateFollowCenter(double lead_x, double lead_y);
    void calculateFormation();
    double getDoubleFromNodeReport(std::string full_string, std::string key);
    std::string getStringFromNodeReport(std::string full_string, std::string key);
    void calcDxDyOperatorsStd(double const spacing, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y);
    void calcDxDyOperators2h(double const spacing, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y);

    // Own util funcs
    double deg2rad(double degrees);
    double rad2deg(double radians);
    double dx(double range, double trig_angle);
    double dy(double range, double trig_angle);
    size_t quadrant(double lead_heading);

    // Configuration variables
    double m_follow_range;
    double m_inter_vehicle_distance;
    std::string m_lead_vehicle;

    // State variables
    double m_allowable_width, m_allowable_height;
    size_t m_num_vehicles;
    double m_follow_center_x, m_follow_center_y;
    std::string m_shape;
    std::string m_prev_shape;
    double m_lead_hdg;
};

#endif 
