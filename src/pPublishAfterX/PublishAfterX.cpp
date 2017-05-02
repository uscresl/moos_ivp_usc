/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: PublishAfterX.cpp                                    */
/*    DATE: Sept 3, 2016                                         */
/*                                                               */
/*****************************************************************/

#include "PublishAfterX.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

#include <ctime>

//---------------------------------------------------------
// Constructor
//
PublishAfterX::PublishAfterX() :
  m_var(""),
  m_val(""),
  m_secs_after(-1),
  m_min_secs(-1),
  m_max_secs(-1),
  m_pub_var(""),
  m_pub_val(""),
  m_start_time(-1.0),
  m_first(true)
{
  // class variable instantiations can go up here

  // seed the random number generator, using
  // current date/time based on current system
  // use minutes to get same seed on multi-robot sim
  time_t now = time(nullptr);
  tm *local_tm = localtime(&now);
  srand(local_tm->tm_min);

}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB,
// there is 'new mail', check to see if
// there is anything for this process.
//
bool PublishAfterX::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  for ( p=NewMail.begin(); p!=NewMail.end(); p++ ) {
    CMOOSMsg &msg = *p;
    std::string key   = msg.GetKey();
    std::string sval  = msg.GetString();
    // separate way for getting the double val (sval was not working for DB_UPTIME)
    //double dval  = msg.GetDouble();

#if 0 // Keep these around just for template
    std::string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if ( key == m_var && m_var != "" )
    {
      if ( (sval == m_val) && (m_secs_after > 0) && (m_start_time < 0) && !m_first )
      {
        m_start_time = MOOSTime();
        std::cout << GetAppName() << " :: setting timer, publishing " << m_secs_after
                  << " seconds from now (" << std::setprecision(15) << m_start_time << ")" << std::endl;
      }
      if ( m_first )
        m_first = false;
      // we skip the first one, because that is coming from pHelmIvP:HELM_VAR_INIT
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool PublishAfterX::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PublishAfterX::Iterate()
{
  if ( m_start_time > 0 )
  {
    if ( (MOOSTime() - m_start_time) > m_secs_after )
    {
      std::cout << GetAppName() << " :: timer expired, publishing: " << m_pub_var << "=" << m_pub_val << std::endl;
      m_Comms.Notify(m_pub_var,m_pub_val);

      // reset
      m_start_time = -1.0;
    }
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PublishAfterX::OnStartUp()
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
    if ( (param == "publish_after_secs") && isNumber(value) )
    {
      m_secs_after = atoi(value.c_str());
      handled = true;
    }
    else if ( (param == "publish_after_secs") && !isNumber(value) )
    {
      // range
      size_t divider = value.find(':');
      if ( divider != std::string::npos )
      {
        m_min_secs = atoi( (value.substr(0, divider)).c_str() );
        m_max_secs = atoi( (value.substr(divider+1, value.length())).c_str() );

        // generate m_secs_after
        int add_random = (int)(rand() % (m_max_secs - m_min_secs));
        m_secs_after = m_min_secs + add_random;

        std::cout << GetAppName() << " :: publish m_secs_after: " << m_secs_after << std::endl;

        handled = true;
      }
    }
    else if ( param == "publish_after_var" )
    {
      // e.g. VAR=value
      m_var = value;
      handled = true;
    }
    else if ( param == "publish_after_val" )
    {
      m_val = value;
      handled = true;
    }
    else if ( param == "publish_var" )
    {
      m_pub_var = value;
      handled = true;
    }
    else if ( param == "publish_val" )
    {
      m_pub_val = value;
      handled = true;
    }
    else if ( param == "skip_first" )
    {
      m_first = ( value == "true" ? true : false );
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
void PublishAfterX::registerVariables()
{
  if ( m_var != "" )
    m_Comms.Register(m_var, 0);
}
