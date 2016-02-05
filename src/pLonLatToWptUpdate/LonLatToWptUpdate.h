/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: LonLatToWptUpdate.h                                  */
/*    DATE: Feb 5, 2016                                          */
/*                                                               */
/*****************************************************************/

#ifndef PLONLATTOWPTUPDATE_HEADER
#define PLONLATTOWPTUPDATE_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class LonLatToWptUpdate : public CMOOSApp
{
  public:
    LonLatToWptUpdate();
    ~LonLatToWptUpdate() {};

  protected:
    // Standard MOOSApp functions to overload
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();

    // Registration, Configuration, Mail handling utils
    void registerVariables();
    bool handleMailLonLatToWptUpdateVarIn(std::string);

  private:
    // Own functions
    void makeYourOwn();

    // Configuration variables
    std::string m_example1;
    double m_example2;

    // State variables
    double m_whatever;
    bool m_got_aabbcc;
};

#endif 
