/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: PublishAfterX.h                                      */
/*    DATE: Sept 3, 2016                                         */
/*                                                               */
/*****************************************************************/

#ifndef PPUBLISHAFTERX_HEADER
#define PPUBLISHAFTERX_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class PublishAfterX : public CMOOSApp
{
  public:
    PublishAfterX();
    ~PublishAfterX() {};

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
    void makeYourOwn();

    // Configuration variables
    std::string m_var;
    std::string m_val;

    int m_secs_after;
    int m_min_secs;
    int m_max_secs;

    std::string m_pub_var;
    std::string m_pub_val;

    // State variables
    double m_start_time;
};

#endif
