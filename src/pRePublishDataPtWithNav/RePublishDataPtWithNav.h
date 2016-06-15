/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: RePublishDataPtWithNav.h                             */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#ifndef PREPUBLISHDATAPTWITHNAV_HEADER
#define PREPUBLISHDATAPTWITHNAV_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class RePublishDataPtWithNav : public CMOOSApp
{
  public:
    RePublishDataPtWithNav();
    ~RePublishDataPtWithNav() {};

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
    std::string appendNrToString(std::string str, size_t nr);

    // Configuration variables
    size_t m_nr_data_pts;
    std::string m_data_var_nm;

    // State variables
    size_t m_data_counter;
    double m_nav_x;
    double m_nav_y;
    double m_nav_depth;

};

#endif 
