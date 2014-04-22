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

#ifndef PTEMPLATE_HEADER
#define PTEMPLATE_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

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

    // Configuration variables

    // State variables
    double m_x, m_y, m_z;
    std::string m_formation;
};

#endif 
