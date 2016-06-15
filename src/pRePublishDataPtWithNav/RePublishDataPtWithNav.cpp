/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: RePublishDataPtWithNav.cpp                           */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#include "RePublishDataPtWithNav.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

//---------------------------------------------------------
// Constructor
//
RePublishDataPtWithNav::RePublishDataPtWithNav() :
  m_nr_data_pts(0),
  m_data_var_nm(""),
  m_data_counter(0),
  m_nav_x(0.0),
  m_nav_y(0.0),
  m_nav_depth(0.0)
{
  // class variable instantiations can go here
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool RePublishDataPtWithNav::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  for ( p=NewMail.begin(); p!=NewMail.end(); p++ ) {
    CMOOSMsg &msg = *p;
    std::string key   = msg.GetKey();
    std::string sval  = msg.GetString();
    // separate way for getting the double val (sval was not working for DB_UPTIME)
    double dval  = msg.GetDouble();

#if 0 // Keep these around just for template
    std::string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    
    if ( key == "NAV_X" )
      m_nav_x = dval;
    else if ( key == "NAV_Y" )
      m_nav_y = dval;
    else if ( key == "NAV_DEPTH" )
      m_nav_depth = dval;
    else if ( key == m_data_var_nm && m_data_var_nm != "" )
    {
      // rewrite data point to DB, labeled, for acomms
      m_data_counter++;
      if ( m_data_counter > m_nr_data_pts )
        m_data_counter = 1;
      m_Comms.Notify(appendNrToString("DATA_X", m_data_counter), m_nav_x);
      m_Comms.Notify(appendNrToString("DATA_Y", m_data_counter), m_nav_y);
      m_Comms.Notify(appendNrToString("DATA_DEPTH", m_data_counter), m_nav_depth);
      m_Comms.Notify(appendNrToString("DATA_VAL", m_data_counter), dval);
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool RePublishDataPtWithNav::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool RePublishDataPtWithNav::Iterate()
{
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RePublishDataPtWithNav::OnStartUp()
{
  CMOOSApp::OnStartUp();
  
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if ( !m_MissionReader.GetConfiguration(GetAppName(), sParams) )
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();

  STRING_LIST::iterator p;
  for ( p=sParams.begin(); p!=sParams.end(); p++ )
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if ( param == "data_var_name" )
    {
      m_data_var_nm = value;
      if ( m_data_var_nm == "" )
      {
        std::cout << GetAppName() << " :: ERROR: no data variable name given" << std::endl;
        RequestQuit();
      }
      else
        std::cout << GetAppName() << " :: setting data_var_name: " << m_data_var_nm << std::endl;
      handled = true;
    }
    else if ( param == "nr_data_points" )
    {
      m_nr_data_pts = atoi(value.c_str());
      if ( m_nr_data_pts < 1 )
      {
        std::cout << GetAppName() << " :: ERROR: nr data points < 1" << std::endl;
        RequestQuit();
      }
      else
        std::cout << GetAppName() << " :: setting nr_data_points: " << m_nr_data_pts << std::endl;
      handled = true;
    }

    if ( !handled )
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
  }

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables
//            at startup, let the MOOSDB know what you want
//            to receive
//
void RePublishDataPtWithNav::registerVariables()
{
  m_Comms.Register("NAV_X", 0);
  m_Comms.Register("NAV_Y", 0);
  m_Comms.Register("NAV_DEPTH", 0);
  if ( m_data_var_nm != "" )
    m_Comms.Register(m_data_var_nm, 0);
}


std::string RePublishDataPtWithNav::appendNrToString(std::string str, size_t nr)
{
  std::ostringstream app_str;
  app_str << str << nr;
  return app_str.str();
}
