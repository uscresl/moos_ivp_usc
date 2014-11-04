/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SelectFormation.cpp                                  */
/*    DATE: Apr 21, 2014                                         */
/*                                                               */
/*    This process will choose what formation should be taken,   */
/*    based on the allowed width/height and number of vehicles   */
/*    And it will output a comma-separated list of positions in  */
/*    the formation.                                             */
/*                                                               */
/*    Nb. this is not an AppCastMOOSApp                          */
/*                                                               */
/*****************************************************************/

#include "SelectFormation.h"

#include <iterator>
#include "MBUtils.h"
#include <limits>
#include <algorithm>

using namespace std;

//---------------------------------------------------------
// Constructor
//
SelectFormation::SelectFormation()
{
  // class variable instantiations can go here
  
  // comms received variables
  m_allowable_height = -1.0;
  m_num_vehicles = 1;
  m_formation_shape = "1AUV";
  
  // parameter value variables
  m_follow_range = 0;
  m_inter_vehicle_distance = 0;
  m_lead_vehicle = "";
  
  // algorithm variables
  m_follow_center_x = 0;
  m_follow_center_y = 0;
  
  debug = true;
  
  m_prev_time = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool SelectFormation::OnNewMail(MOOSMSG_LIST &NewMail)
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
    bool new_info = false;
    if( key == "ALLOWABLE_WIDTH" )
    {
      processReceivedWidth(dval);
      new_info = true;
    }
    else if ( key == "ALLOWABLE_HEIGHT" )
    {
      m_allowable_height = dval;
      new_info = true;
    }
    else if ( key == "NAV_SPEED" )
    {
      m_own_spd = dval;
    }
    else if ( key == "NUM_VEHICLES" )
    {
      m_num_vehicles = (size_t)round(dval);
      new_info = true;
    }
    else if ( key == "NODE_REPORT" )
    {
      // check if right vehicle
      std::string veh_name = getStringFromNodeReport(sval, "NAME");
      if ( veh_name == m_lead_vehicle )
      {
        // need speed to calculate what time corresponds to follow_range
        double lead_spd, update_time;
        lead_spd = getDoubleFromNodeReport(sval, "SPD");
        update_time = getDoubleFromNodeReport(sval, "TIME");

        // store info: add with timestamp of message
        // MOOSTime is warped, so should be ok as key (= DB_TIME)
        LeadHistory update;
        update.timestamp = update_time;
        update.node_report = sval;
        
        if ( m_lead_history.size() == 0 || (m_lead_history.back().timestamp < update.timestamp ) )
        { // just insert, assume rest is already sorted
          m_lead_history.insert(m_lead_history.end(), update);
        }
        else
        { // insert at lower_bound, keeps vector sorted
          m_lead_history.insert(std::lower_bound(m_lead_history.begin(), m_lead_history.end(), update), update);
        }

        // update the formation center/reference point 
        updateFollowCenter(update_time, lead_spd);
        new_info = true;
      }
    }
    else
      std::cout << "pSelectFormation :: Unhandled Mail: " << key << std::endl;
      //reportRunWarning("Unhandled Mail: " + key);
    
    // any of these updates will require recalculation of formation positions,
    // so let's recalculate (since not all are updated at same time, values
    // are stored as class vars).
    if ( new_info )
      calculateFormation();
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SelectFormation::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SelectFormation::Iterate()
{
  updateFormationShape();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SelectFormation::OnStartUp()
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
    if ( (param == "follow_range") && isNumber(value) )
    {
      // assuming the atof works, store the value
      m_follow_range = atof(value.c_str());
      handled = true;

      std::cout << GetAppName() << " :: set m_follow_range to be: " << m_follow_range << std::endl;
    }
    else if ( (param == "inter_vehicle_distance") && isNumber(value) ) 
    {
      // assuming the atof works, store the value
      m_inter_vehicle_distance = atof(value.c_str());
      handled = true;

      std::cout << GetAppName() << " :: set m_inter_vehicle_distance to be: " << m_follow_range << std::endl;
    }
    else if ( param == "lead_vehicle_name" )
      m_lead_vehicle = value;

    if ( !handled )
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
void SelectFormation::registerVariables()
{
  m_Comms.Register("ALLOWABLE_WIDTH", 0);
  m_Comms.Register("ALLOWABLE_HEIGHT", 0);
  m_Comms.Register("NUM_VEHICLES", 0);
  m_Comms.Register("NODE_REPORT", 0);
  m_Comms.Register("NAV_SPEED", 0);
}

void SelectFormation::updateFollowCenter(double const curr_time, double const lead_spd)
{
  double trig_angle;
  double delta_x, delta_y;
  bool pos_x = true, pos_y = true;

  // convert range to time, to extract prev location lead from memory
  double time_behind = m_follow_range / lead_spd;

  // extract prev position lead from memory
  double time_key = curr_time - time_behind;
  std::vector<LeadHistory>::iterator lower_bound;
  LeadHistory tmp;
  tmp.timestamp = time_key;
  lower_bound = std::lower_bound(m_lead_history.begin(), m_lead_history.end(), tmp);

  // if valid data is available, update the follow center
  if ( lower_bound != m_lead_history.end() )
  { // only take data if element exists in vector
    double timestamp = (*lower_bound).timestamp;
    std::string node_report = (*lower_bound).node_report;

    m_follow_center_x = getDoubleFromNodeReport(node_report,"X");
    m_follow_center_y = getDoubleFromNodeReport(node_report,"Y");
    m_lead_hdg = getDoubleFromNodeReport(node_report,"HDG");
  }
  // should I add something in case it doesn't exist (shouldn't happen?)?

  // show on pMarineViewer
  std::ostringstream ctr_pt;
  ctr_pt << "x=" << m_follow_center_x << ",y=" << m_follow_center_y 
         << ",label=follow_center";
  Notify("VIEW_POINT",ctr_pt.str());
}

void SelectFormation::updateFormationShape()
{
  // not tested yet: what if information received late?
  // test & adapt when adding full acomms
  size_t curr_time = round(MOOSTime());
  if ( m_formation_shape_map.find(curr_time) != m_formation_shape_map.end() )
  { // found an update, update global var
    m_formation_shape = m_formation_shape_map.at(curr_time);
    std::cout << "Changing shape to: " << m_formation_shape << std::endl;
    Notify("FORMATION_SHAPE", m_formation_shape);
    // TODO: don't let the map get humongous, erase old items
  }
}


//---------------------------------------------------------
// Procedure: calculateFormation
//            calculate which formation possible given
//            allowable width/height and nr vehicles
//
//            for now, kind of link a bank of options, but
//            it would be nice to generate these..
//
void SelectFormation::calculateFormation()
{
//  m_prev_shape = m_shape;

  // for now, only X/Y
  std::ostringstream formation_string;
  double x1, y1, x2, y2, x3, y3;
  if ( m_formation_shape == "1AUV" )
    formation_string << m_follow_center_x << "," << m_follow_center_y;
  else if ( m_formation_shape == "2AUVh" )
  { // horizontal: 2 vehicles parallel to each other
    double delta_x, delta_y;
    bool pos_x1 = true, pos_y1 = true; // for vehicle 2 is inverse
    calcDxDyOperators2h(m_inter_vehicle_distance/2, m_lead_hdg, delta_x, delta_y, pos_x1, pos_y1);
    x1 = ( pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
    x2 = ( !pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
    y1 = ( pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );
    y2 = ( !pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );

    if (debug)
    {
      std::ostringstream pt1;
      pt1 << "x=" << x1 << ",y=" << y1 << ",label=p1";
      Notify("VIEW_POINT",pt1.str());
      std::ostringstream pt2;
      pt2 << "x=" << x2 << ",y=" << y2 << ",label=p2";
      Notify("VIEW_POINT",pt2.str());
    }
  }
  else if ( m_formation_shape == "2AUVv" )
  { // vertical: 2 vehicles behind one another
    double delta_x, delta_y;
    bool pos_x = true, pos_y = true; // only for 2nd vehicle
    calcDxDyOperatorsStd(m_inter_vehicle_distance, m_lead_hdg, delta_x, delta_y, pos_x, pos_y);
    x1 = m_follow_center_x;
    x2 = ( pos_x ? x1 + delta_x : x1 - delta_x );
    y1 = m_follow_center_y;
    y2 = ( pos_y ? y1 + delta_y : y1 - delta_y );

    if (debug)
    {
      std::ostringstream pt1;
      pt1 << "x=" << x1 << ",y=" << y1 << ",label=p1";
      Notify("VIEW_POINT",pt1.str());
      std::ostringstream pt2;
      pt2 << "x=" << x2 << ",y=" << y2 << ",label=p2";
      Notify("VIEW_POINT",pt2.str());
    }
  }
  else if ( m_formation_shape == "3AUVh" )
  {
    // horizontal
    // for the two outside vehicles, calculate offsets
    double delta_x, delta_y;
    bool pos_x1 = true, pos_y1 = true;
    calcDxDyOperators2h(m_inter_vehicle_distance, m_lead_hdg, delta_x, delta_y, pos_x1, pos_y1);

    x1 = ( pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
    x3 = ( !pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
    y1 = ( pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );
    y3 = ( !pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );
    x2 = m_follow_center_x;
    y2 = m_follow_center_y;
  }
  else if ( m_formation_shape == "3AUVm" )
  { // 1 front, 2 back TODO trig
    x1 = m_follow_center_x;
    y1 = m_follow_center_y;
    x2 = m_follow_center_x - m_inter_vehicle_distance;
    y2 = m_follow_center_y - m_inter_vehicle_distance;
    x3 = m_follow_center_x + m_inter_vehicle_distance;
    y3 = y2;
  }
  else if ( m_formation_shape == "3AUVv" )
  { // vertical
    double delta_x, delta_y;
    bool pos_x = true, pos_y = true; // only for 2nd vehicle
    calcDxDyOperatorsStd(m_inter_vehicle_distance, m_lead_hdg, delta_x, delta_y, pos_x, pos_y);

    x1 = m_follow_center_x;
    y1 = m_follow_center_y;
    x2 = ( pos_x ? x1 + delta_x : x1 - delta_x );
    y2 = ( pos_y ? y1 + delta_y : y1 - delta_y );
    x3 = ( pos_x ? x1 + 2*delta_x : x1 - 2*delta_x );
    y3 = ( pos_y ? y1 + 2*delta_y : y1 - 2*delta_y );
  }

  if ( m_formation_shape.at(0) == '2' )
    formation_string << x1 << "," << y1 << ":"  << x2 << "," << y2;
  else if ( m_formation_shape.at(0) == '3' )
  {
    formation_string << x1 << "," << y1 << ":" 
                     << x2 << "," << y2 << ":" 
                     << x3 << "," << y3;
  }

  // notify
  Notify("DESIRED_FORMATION",formation_string.str());
}

void SelectFormation::processReceivedWidth(double const allowable_width)
{
  // convert follow range to time, to know when to start changing
  size_t add_lag = round(m_follow_range / m_own_spd);
  // nb. time message received != time message sent. 
  // test/adapt when adding full acomms
  size_t current_time = round(MOOSTime());
  size_t start_time = current_time+add_lag;//was: 1.5*lag
  
  std::string new_shape;
  switch ( m_num_vehicles)
  {
    case 1:
      new_shape = "1AUV";
      break;
    case 2:
      if ( allowable_width >= m_inter_vehicle_distance )
        new_shape = "2AUVh";
      else
        new_shape = "2AUVv";
      break;
    case 3:
      if ( allowable_width >= 2*m_inter_vehicle_distance )
        new_shape = "3AUVh";
      else if ( allowable_width >= m_inter_vehicle_distance )
        new_shape = "3AUVm";
      else
        new_shape = "3AUVv";
      break;
    default:
      new_shape = "2AUVv";
      break;
  }
  
  // if the determined shape is different than last received, add it to the map
  if ( m_formation_shape_map.size() == 0 )
  { // nothing in map yet, always add
    m_formation_shape_map.insert(std::pair<size_t,std::string>(start_time,new_shape));
  }
  else
  {
    std::map<size_t,std::string>::iterator last_in_map = m_formation_shape_map.end();
    last_in_map--;
    std::string last_shape = last_in_map->second;
    if ( last_shape != new_shape )
    { // need to change shape, so store
      m_formation_shape_map.insert(std::pair<size_t,std::string>(start_time,new_shape));
    }
  }
}
