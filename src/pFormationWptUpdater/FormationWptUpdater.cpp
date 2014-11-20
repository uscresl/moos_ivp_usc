/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: FormationWptUpdater.cpp                              */
/*    DATE: Apr 21, 2014                                         */
/*                                                               */
/*    This process will publish an update for a waypoint         */
/*    behavior, based on the formation and position in formation */
/*                                                               */
/*****************************************************************/

#include "FormationWptUpdater.h"

#include <iterator>
#include "MBUtils.h"
#include "math.h"
#include <limits>

using namespace std;

//---------------------------------------------------------
// Constructor
//
FormationWptUpdater::FormationWptUpdater()
{
  // class variable instantiations can go here
  m_position = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool FormationWptUpdater::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    string sval  = msg.GetString();
    // separate way for getting the double val (sval was not working for DB_UPTIME) 
    double dval  = msg.GetDouble();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if ( key == "DESIRED_FORMATION" )
      sendWaypoint(sval);
    else if ( key == "POSITION_IN_FORMATION" )
    {
      m_position = (size_t)round(dval);
      std::cout << "received position: " << dval << std::endl;
    }
    else
      std::cout << "pFormationWptUpdater :: Unhandled Mail: " << key << std::endl;
      //reportRunWarning("Unhandled Mail: " + key);
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool FormationWptUpdater::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool FormationWptUpdater::Iterate()
{
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool FormationWptUpdater::OnStartUp()
{
  CMOOSApp::OnStartUp();
  
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();
    //reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) 
  {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;

    if(!handled)
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
      //reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables
//            at startup, let the MOOSDB know what you want
//            to receive
//
void FormationWptUpdater::registerVariables()
{
  m_Comms.Register("DESIRED_FORMATION", 0);
  m_Comms.Register("POSITION_IN_FORMATION", 0);
}

void FormationWptUpdater::sendWaypoint( std::string formation_string )
{
  // store globally: desired position
  // need to grab this from the formation string and publish as wpt update
  vector<string> svector = parseString(formation_string, ':');
  unsigned int vsize = svector.size();
  if ( m_position > vsize )
    std::cout << "impossible situation" << std::endl;
  else
  {
    if ( m_position >= 1 ) // avoid sending something before valid position
    {
      std::ostringstream output;
      output << "points=" << svector[m_position-1];
      Notify("FORMATION_WAYPOINT_UPDATE",output.str());
    }
  }
}
