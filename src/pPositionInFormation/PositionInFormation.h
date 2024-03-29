/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: PositionInFormation.cpp                              */
/*    DATE: Apr 21, 2014                                         */
/*                                                               */
/*                                                               */
/*    This process will choose what position in the formation    */
/*    the vehicles should take, based on which one is closest.   */
/*                                                               */
/*    Nb. this is not an AppCastMOOSApp                          */
/*                                                               */
/*****************************************************************/

#ifndef PPOSITION_IN_FORMATION_HEADER
#define PPOSITION_IN_FORMATION_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MBUtils.h"
#include "math.h"
#include "USCutils.h"

class PositionInFormation : public CMOOSApp
{
  public:
    PositionInFormation();
    ~PositionInFormation() {};

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
    void findPosition();
    void euclidDistanceFromString(std::string const & xy_str, double vehicle_x, double vehicle_y, double & euclidD);
    void cleanVehicleMap();

    // Configuration variables
    std::string m_lead_vehicle;

    // State variables
    double m_x, m_y;
//    , m_z;
    std::string m_formation;
    std::map<std::string,std::string> m_other_vehicles;
    std::map<std::string,double> m_other_vehicles_update_time;
    double m_time_limit;
    std::string m_ownship;
    size_t m_nr_followers;
    bool debug;
};

#endif 
