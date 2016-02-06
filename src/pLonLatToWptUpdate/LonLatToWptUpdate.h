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
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

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
    void handleMailLonLatToWptUpdateVarIn(std::string);

  private:
    // Own functions
    void initGeodesy();
    bool lonLatToUTM (double lon, double lat, double & lx, double & ly );
    void publishWpt(double lx, double ly);

    // Configuration variables
    std::string m_input_var_lonlat;
    std::string m_output_var_wpt_update;

    // State variables

    // Class variables
    CMOOSGeodesy m_geodesy;
};

#endif 
