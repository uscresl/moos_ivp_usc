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

    // util funcs, TODO move to own library
    double getDoubleFromNodeReport(std::string full_string, std::string key);
    std::string getStringFromNodeReport(std::string full_string, std::string key);
    void euclidDistance(double const x1, double const y1, double const x2, double const y2, double & euclid);

    // Configuration variables
    std::string m_lead_vehicle;

    // State variables
    double m_x, m_y, m_z;
    std::string m_formation;
    std::map<std::string,std::string> m_other_vehicles;
    std::string m_ownship;
};

#endif 
