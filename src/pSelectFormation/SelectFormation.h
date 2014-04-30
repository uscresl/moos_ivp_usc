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
#include "USCutils.h"
#include <vector>

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
    void updateFollowCenter(double curr_time, double lead_spd); //double lead_x, double lead_y, double lead_spd);
    void calculateFormation();

    // Configuration variables
    double m_follow_range;
    double m_inter_vehicle_distance;
    std::string m_lead_vehicle;

    // State variables
    bool debug;
    double m_allowable_width, m_allowable_height;
    size_t m_num_vehicles;
    double m_follow_center_x, m_follow_center_y;
    std::string m_shape;
    std::string m_prev_shape;
    double m_lead_hdg;
    
    double m_prev_time;
    
    struct LeadHistory {
      double timestamp;
      std::string node_report;
      bool operator<(const LeadHistory &other) const { return timestamp < other.timestamp; };
    };
    std::vector<LeadHistory> m_lead_history;
};

#endif 
