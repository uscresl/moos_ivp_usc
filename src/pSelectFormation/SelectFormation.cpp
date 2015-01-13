/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SelectFormation.cpp                                  */
/*    DATE: Apr 21, 2014, last updated: Nov 20, 2014             */
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

// matrix operations
#include <Eigen/Dense>

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
  m_previous_formation_string = "";
  m_last_shape = "";
  m_lead_spd = 0;

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
      // this comes via acomms, and includes time, which needs to be
      // read, and passed on to the method that processes it, for accurate
      // estimation of when to switch formation.
      processReceivedWidth(sval);
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
      std::cout << GetAppName() << " :: m_num_vehicles set to: " << m_num_vehicles << std::endl;
    }
    else if ( key == "NODE_REPORT" )
    {
      // check if right vehicle
      std::string veh_name = getStringFromNodeReport(sval, "NAME");
      if ( veh_name == m_lead_vehicle )
      {
        // need speed to calculate what time corresponds to follow_range
        new_info = getInfoFromNodeReport(sval);
      }
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;
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

      std::cout << GetAppName() << " :: set m_inter_vehicle_distance to be: " << m_inter_vehicle_distance << std::endl;
    }
    else if ( param == "lead_vehicle_name" )
    {
      m_lead_vehicle = value;
      handled = true;
      std::cout << GetAppName() << " :: set m_lead_vehicle to be: " << m_lead_vehicle << std::endl;
    }
    else if ( param == "lead_sensor_range")
    {
      m_lead_sensor_range = atof(value.c_str());
      handled = true;
      std::cout << GetAppName() << " :: set m_lead_sensor_range to be: " << m_lead_sensor_range << std::endl;
    }

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

//---------------------------------------------------------
// Procedure: updateFollowCenter
//            after storing node_report into leadhistory,
//            calculate where follow center is along
//            leadhistory trajectory.
//
//  Given that the exact time may not be in the leadhistory,
//  it will take the std::lower_bound:
//  the first element in the range [begin,end) which does not compare less than val (tmp).
//
void SelectFormation::updateFollowCenter(double const curr_time, double const lead_spd)
{
  double trig_angle;
  double delta_x, delta_y;
  bool pos_x = true, pos_y = true;

  // convert range to time, to extract prev location lead from memory
  // we want to know the location of the lead not at the current time, but
  // when it was where we are supposed to be.
  double time_behind = m_follow_range / lead_spd;

  // extract prev position lead from memory
  double time_key = curr_time - time_behind;
  std::vector<LeadHistory>::iterator closest_val;
  LeadHistory tmp;
  tmp.timestamp = time_key;
  closest_val = std::lower_bound(m_lead_history.begin(), m_lead_history.end(), tmp);
  // if valid data is available, update the follow center
  double timestamp = -1;
  std::string node_report = "";
  if ( closest_val != m_lead_history.end() )
  { // only take data if element exists in vector
    timestamp = (*closest_val).timestamp;
    node_report = (*closest_val).node_report;
  }
  else if ( m_lead_history.size() != 0 )
  {
    // if there are only older values, take the last old one
    timestamp = (*(m_lead_history.end()-1)).timestamp;
    node_report = (*(m_lead_history.end()-1)).node_report;
  }
  if ( timestamp > 0 )
  {
    m_follow_center_x = getDoubleFromNodeReport(node_report,"X");
    m_follow_center_y = getDoubleFromNodeReport(node_report,"Y");
    m_lead_hdg = getDoubleFromNodeReport(node_report,"HDG");
  }

  // show on pMarineViewer
  std::ostringstream ctr_pt;
  ctr_pt << "x=" << m_follow_center_x << ",y=" << m_follow_center_y 
         << ",label=follow_center";
  Notify("VIEW_POINT",ctr_pt.str());
}

void SelectFormation::updateFormationShape()
{
  // check if formation shape needs changing at current time
  if ( m_formation_shape_map.size() > 0 )
  {
    size_t curr_time = round(MOOSTime());
    std::map<size_t,std::string>::iterator itx;
    std::map<size_t,std::string>::iterator l_bound;
    itx = m_formation_shape_map.find(curr_time);
    l_bound = m_formation_shape_map.lower_bound(curr_time);
    bool retrieved = false;
    if ( itx != m_formation_shape_map.end() )
    { // found an update, update global var
      m_formation_shape = m_formation_shape_map.at(curr_time);
      retrieved = true;
    }
    else if ( l_bound != m_formation_shape_map.begin() )
    {  // catch the case that there are older ones, eg due to acomms delay
       m_formation_shape = m_formation_shape_map.begin()->second;
       retrieved = true;
    }
    if ( retrieved )
    {
      // tell the world
      std::cout << GetAppName() << " :: Changing shape to: " << m_formation_shape << std::endl;
      Notify("FORMATION_SHAPE", m_formation_shape);
      // don't let the map get humongous, erase published items
      m_formation_shape_map.erase(itx);
      // also make sure we redo calculations
      calculateFormation();
    }
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
  // update the formation center/reference point
  size_t curr_time = round(MOOSTime());
  updateFollowCenter(curr_time, m_lead_spd);

  // for now, 2D: only X/Y
  std::ostringstream formation_string;
  double x1, y1, x2, y2, x3, y3;
  if ( m_formation_shape == "1AUV" )
  {
    x1 = m_follow_center_x;
    y1 = m_follow_center_y;
    formation_string << x1 << "," << y1;
  }

  // if more than 1 vehicle
  if ( m_formation_shape.at(0) == '2' || m_formation_shape.at(0) == '3' )
  {
    // initiate rotation and translation matrices

    // Eigen typedef shortcuts used here:
    //  typedef Matrix< double, 3, 1 > 	Vector3d
    //  typedef Matrix< double, 3, 3 > 	Matrix3d
    Eigen::Vector3d follow_ctr;
    follow_ctr(0) = m_follow_center_x;
    follow_ctr(1) = m_follow_center_y;
    follow_ctr(2) = 1;
//    std::cout << "\nFOLLOW CENTER" << std::endl;
//    std::cout << follow_ctr << std::endl;

    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    double hdg_rad = deg2rad(m_lead_hdg);
//    std::cout << "heading deg: " << m_lead_hdg << " and rad: " << hdg_rad << std::endl;
    rot(0,0) = std::cos(hdg_rad);
    rot(0,1) = std::sin(hdg_rad);
    rot(1,0) = -1*std::sin(hdg_rad);
    rot(1,1) = std::cos(hdg_rad);
//    std::cout << "ROTATION MATRIX" << std::endl;
//    std::cout << rot << std::endl;

    Eigen::Vector3d trans_auv1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d trans_auv2 = Eigen::Vector3d::Zero();
    trans_auv1(2) = 1;
    trans_auv2(2) = 1;

    Eigen::Vector3d auv1;
    Eigen::Vector3d auv2;

    if ( m_formation_shape.at(0) == '2' )
    {
      if ( m_formation_shape.at(4) == 'h' )
      { // horizontal: 2 vehicles parallel to each other
        trans_auv1(0) = -(m_inter_vehicle_distance/2);
        trans_auv2(0) = m_inter_vehicle_distance/2;
      }
      else if ( m_formation_shape.at(4) == 'v' )
      { // vertical: 2 vehicles behind one another
        trans_auv2(1) = -m_inter_vehicle_distance;
      }
      // calculate positions and format into string

      auv1 = follow_ctr + (rot*trans_auv1);
      auv2 = follow_ctr + (rot*trans_auv2);
//      std::cout << "POSITION AUV1" << std::endl;
//      std::cout << auv1 << std::endl;
      formation_string << auv1(0) << "," << auv1(1) << ":"
                       << auv2(0) << "," << auv2(1);
//      std::cout << "OUTPUT STRING" << std::endl;
//      std::cout << formation_string.str() << std::endl;
    }
    else if ( m_formation_shape.at(0) == '3' )
    {
      // add 3rd auv
      Eigen::Vector3d trans_auv3 = Eigen::Vector3d::Zero();
      trans_auv3(2) = 1;
      if ( m_formation_shape == "3AUVh" )
      { // horizontal
        // for the two outside vehicles, calculate offsets
        trans_auv1(0) = -1*m_inter_vehicle_distance;
        trans_auv3(0) = m_inter_vehicle_distance;
      }
      else if ( m_formation_shape == "3AUVm" )
      { // 1 front, 2 back TODO trig
        trans_auv2(0) = -(m_inter_vehicle_distance/2);
        trans_auv2(1) = -m_inter_vehicle_distance;
        trans_auv3(0) = m_inter_vehicle_distance/2;
        trans_auv3(1) = -m_inter_vehicle_distance;
      }
      else if ( m_formation_shape == "3AUVv" )
      { // vertical
        trans_auv2(1) = -m_inter_vehicle_distance;
        trans_auv3(1) = -2*m_inter_vehicle_distance;
      }
      // calculate positions and format into string
      auv1 = follow_ctr + (rot*trans_auv1);
      auv2 = follow_ctr + (rot*trans_auv2);
      Eigen::Vector3d auv3;
      auv3 = follow_ctr + (rot*trans_auv3);
      x3 = auv3(0);
      y3 = auv3(1);
      formation_string << auv1(0) << "," << auv1(1) << ":"
                       << auv2(0) << "," << auv2(1) << ":"
                       <<  auv3(0) << "," << auv3(1);
    }
    x1 = auv1(0);
    y1 = auv1(1);
    x2 = auv2(0);
    y2 = auv2(1);
  }

  // notify, but only if we have a new position
  std::string new_formation_string = formation_string.str();
  if ( new_formation_string != m_previous_formation_string )
  {
    Notify("DESIRED_FORMATION", new_formation_string);
    m_previous_formation_string = new_formation_string;

    if (debug)
    {
      std::ostringstream pt1;
      pt1 << "x=" << x1 << ",y=" << y1 << ",label=p1";
      Notify("VIEW_POINT",pt1.str());
      if ( m_formation_shape.at(0) == '2' || m_formation_shape.at(0) == '3')
      {
        std::ostringstream pt2;
        pt2 << "x=" << x2 << ",y=" << y2 << ",label=p2";
        Notify("VIEW_POINT",pt2.str());
      }
      if ( m_formation_shape.at(0) == '3' )
      {
        std::ostringstream pt3;
        pt3 << "x=" << x3 << ",y=" << y3 << ",label=p3";
        Notify("VIEW_POINT",pt3.str());
      }
    }
  }
}


void SelectFormation::processReceivedWidth(std::string allowable_width)
{
  // example allowable_width msg content:
  //  UTC_TIME=1416443775.000000000000000,ALLOWABLE_WIDTH=0
  size_t sent_time;
  double ok_width;
  sent_time = round(getDoubleFromCommaSeparatedString(allowable_width, "UTC_TIME"));
  ok_width = getDoubleFromCommaSeparatedString(allowable_width, "ALLOWABLE_WIDTH");

  std::string nan_test = getStringFromNodeReport(allowable_width,"ALLOWABLE_WIDTH");
  if ( nan_test != "nan" )
  {
    // estimate when formation should take this shape, given distance between
    //   lead and formation, and lead sensor range:
    //   convert the follow range to time, to know when to start changing
    size_t add_lag = round( (m_follow_range + m_lead_sensor_range) / m_lead_spd);
    size_t start_time = sent_time + add_lag;

    std::string new_shape = "";
    switch ( m_num_vehicles )
    {
      case 1:
        new_shape = "1AUV";
        break;
      case 2:
        if ( ok_width >= m_inter_vehicle_distance )
          new_shape = "2AUVh";
        else
          new_shape = "2AUVv";
        break;
      case 3:
        if ( ok_width >= 2*m_inter_vehicle_distance )
          new_shape = "3AUVh";
        else if ( ok_width >= m_inter_vehicle_distance )
          new_shape = "3AUVm";
        else
          new_shape = "3AUVv";
        break;
      default:
        new_shape = "2AUVv";
        break;
    }

    // if the determined shape is different than last received, add it to the map
    if ( new_shape != m_last_shape )
    { // need to change shape, so store
      m_formation_shape_map.insert(std::pair<size_t,std::string>(start_time,new_shape));
      m_last_shape = new_shape;
    }
  }
}

bool SelectFormation::getInfoFromNodeReport(std::string sval)
{
  // Example NODE_REPORT via acomms:
  // NAME=anna,TYPE=ship,UTC_TIME=1416433913.000000000000000,X=2700,Y=1900,LAT=34.26372545698,LON=-117.17479031395,SPD=0,HDG=180,DEPTH=0,ALTITUDE=0,PITCH=0,ROLL=0
  double lead_spd, update_time;
  lead_spd = getDoubleFromNodeReport(sval, "SPD");
  update_time = getDoubleFromNodeReport(sval, "UTC_TIME");

  // check that values retrieved correctly
  if ( lead_spd != std::numeric_limits<double>::max() && update_time != std::numeric_limits<double>::max() )
  {
    // store info: add with timestamp of message
    // MOOSTime is warped, so should be ok as key (= DB_TIME)
    LeadHistory update;
    update.timestamp = update_time;
    update.node_report = sval;
    m_lead_spd = lead_spd;

    if ( m_lead_history.size() == 0 || (m_lead_history.back().timestamp < update.timestamp ) )
    { // just insert, assume rest is already sorted
      m_lead_history.insert(m_lead_history.end(), update);
    }
    else
    { // insert at lower_bound, keeps vector sorted
      m_lead_history.insert(std::lower_bound(m_lead_history.begin(), m_lead_history.end(), update), update);
    }
    return true;
  }
  else
    return false;
}
