/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: NumVehicles.cpp                                      */
/*    DATE: Feb 25, 2015                                         */
/*                                                               */
/*    This process publishes the total number of vehicles for    */
/*    which node reports are received, and the names of the      */
/*    other vehicles.                                            */
/*                                                               */
/*****************************************************************/

#include "NumVehicles.h"

#include <iterator> // iterator, std::next
#include "MBUtils.h"
//#include "ACTable.h"
#include "math.h"
#include <limits>
#include "USCutils.h"

//---------------------------------------------------------
// Constructor
//
NumVehicles::NumVehicles()
{
  // class variable instantiations can go here
  m_time_limit = 300; // 300 seconds = 5 minutes
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool NumVehicles::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
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
    
    if( key == "NODE_REPORT" )
    {
      handleNodeReport(sval);
      publishNrVehicles();
    }
    else
      std::cout << "pNumVehicles :: Unhandled Mail: " << key << std::endl;
      //reportRunWarning("Unhandled Mail: " + key);
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool NumVehicles::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool NumVehicles::Iterate()
{
  // remove old node reports, if not recently updated
  cleanVehicleMap();

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool NumVehicles::OnStartUp()
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
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if((param == "time_limit") && isNumber(value))
    {
      // assuming the atof works, store the val
      m_time_limit = atof(value.c_str());
      handled = true;

      std::cout << GetAppName() << " :: set m_time_limit to be: " << m_time_limit << std::endl;
    }

    if(!handled)
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
      //reportUnhandledConfigWarning(orig);
  }

  if(!m_MissionReader.GetValue("Community", m_own_name))
    std::cout << GetAppName() << " :: Unable to read community name." << std::endl;

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables
//            at startup, let the MOOSDB know what you want
//            to receive
//
void NumVehicles::registerVariables()
{
  m_Comms.Register("NODE_REPORT", 0);
}

//---------------------------------------------------------
// Procedure: handleNodeReport
//            a place to do more advanced handling of the
//            incoming message
//
bool NumVehicles::handleNodeReport(std::string const & node_report)
{
  bool handled = false;
  // check what vehicle we got an update from
  std::string vname = getStringFromNodeReport(node_report, "NAME");
  double sent_time = getDoubleFromNodeReport(node_report, "UTC_TIME");

  // we filter out our own - typically we would not receive this, but just in
  // case we do, filter it out.
  if ( vname.compare(m_own_name) != 0 )
  {
    // if not yet in map, add
    std::map<std::string,double>::iterator veh_pos_map = m_other_vehicles.find(vname);
    if ( veh_pos_map == m_other_vehicles.end() )
    {
      m_other_vehicles.insert(std::pair<std::string,double>(vname,sent_time));
      handled = true;
    }
    // else, update time
    else
    {
      veh_pos_map->second = sent_time;
      handled = true;
    }

    if ( !handled )
      std::cout << GetAppName() << " :: Unhandled NODE_REPORT: " << node_report << std::endl;
  }
  else
    handled = true;

  return handled;
}

void NumVehicles::cleanVehicleMap()
{
  size_t curr_time = round(MOOSTime());
  std::map<std::string,double>::iterator veh_iter;
  bool changes_made = false;
  for ( veh_iter = m_other_vehicles.begin(); veh_iter != m_other_vehicles.end(); ++veh_iter )
  {
    if ( curr_time - veh_iter->second > m_time_limit )
    {
      m_other_vehicles.erase(veh_iter);
      changes_made = true;
    }
  }
  if ( changes_made )
    publishNrVehicles();
}

void NumVehicles::publishNrVehicles()
{
  // publish nr of ALL vehicles (including self)
  Notify("NUM_VEHICLES",m_other_vehicles.size()+1);

  // publish names of OTHER vehicles (not self)
  std::ostringstream veh_names;
  std::map<std::string,double>::iterator veh_iter;
  for ( veh_iter = m_other_vehicles.begin(); veh_iter != m_other_vehicles.end(); )//++veh_iter )
  {
    veh_names << veh_iter->first;
    if ( ++veh_iter != m_other_vehicles.end() )
      veh_names << ",";
  }
  std::string output = veh_names.str();
  Notify("OTHER_VEHICLES",output);
}
