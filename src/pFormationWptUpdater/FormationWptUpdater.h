/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: FormationWptUpdater.h                                           */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*    Note: this is a template dir, so you can copy to start     */
/*          making your MOOSApp                                  */
/*                                                               */
/*****************************************************************/

#ifndef PFORMATION_WPT_UPDATER_HEADER
#define PFORMATION_WPT_UPDATER_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class FormationWptUpdater : public CMOOSApp
{
  public:
    FormationWptUpdater();
    ~FormationWptUpdater() {};

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
    void sendWaypoint( std::string formation_string );

    // Configuration variables

    // State variables
    size_t m_position;

};

#endif 
