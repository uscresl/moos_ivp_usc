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

using namespace std;

//---------------------------------------------------------
// Constructor
//
SelectFormation::SelectFormation()
{
  // class variable instantiations can go here
  
  // comms received variables
  m_allowable_width = -1.0;
  m_allowable_height = -1.0;
  m_num_vehicles = 1;
  
  // parameter value variables
  m_follow_range = 0;
  m_inter_vehicle_distance = 0;
  m_lead_vehicle = "";
  
  // algorithm variables
  m_follow_center_x = 0;
  m_follow_center_y = 0;
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
      m_allowable_width = dval;
      new_info = true;
    }
    else if ( key == "ALLOWABLE_HEIGHT" )
    {
      m_allowable_height = dval;
      new_info = true;
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
        double lead_x, lead_y, lead_hdg;
        // need contact thingy to extract position of any name from node report
        lead_x = getDoubleFromNodeReport(sval, "X");
        lead_y = getDoubleFromNodeReport(sval, "Y");
        m_lead_hdg = getDoubleFromNodeReport(sval, "HDG");
        // update the formation center/reference point 
        updateFollowCenter(lead_x, lead_y);
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
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SelectFormation::OnStartUp()
{
  std::cout << "enter OnStartUp" << std::endl;
  CMOOSApp::OnStartUp();

  std::cout << "check leading" << std::endl;
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();
    //reportConfigWarning("No config block found for " + GetAppName());

  std::cout << "start for" << std::endl;
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) 
  {
    std::cout << "iter: " << *p << std::endl;
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
}

void SelectFormation::updateFollowCenter(double lead_x, double lead_y)
{
  double trig_angle;
  double delta_x, delta_y;
  bool pos_x = true, pos_y = true;
  calcDxDyOperatorsStd(m_follow_range, delta_x, delta_y, pos_x, pos_y);
  
  // calculate new follow center
  // need to m_follow_range behind, given hdg
  m_follow_center_x = ( pos_x ? lead_x + delta_x : lead_x - delta_x );
  m_follow_center_y = ( pos_y ? lead_y + delta_y : lead_y - delta_y );

  // show on pMarineViewer
  std::ostringstream ctr_pt;
  ctr_pt << "x=" << m_follow_center_x << ",y=" << m_follow_center_y 
         << ",label=follow_center";
  Notify("VIEW_POINT",ctr_pt.str());
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
  m_prev_shape = m_shape;
  // for now, only X/Y
  std::ostringstream formation_string;
  switch ( m_num_vehicles )
  {
    case 1:
      // stringify
      formation_string << m_follow_center_x << "," << m_follow_center_y;
      m_shape = "1AUV";
      break;
    case 2:
      // 2 AUVs, 2 formations
      double x1, y1, x2, y2;
      if ( m_allowable_width >= m_inter_vehicle_distance )
      { // horizontal: 2 vehicles parallel to each other
        m_shape = "2AUVh";

        double delta_x, delta_y;
        bool pos_x1 = true, pos_y1 = true; // for vehicle 2 is inverse
        calcDxDyOperators2h(m_inter_vehicle_distance/2, delta_x, delta_y, pos_x1, pos_y1);

        x1 = ( pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
        x2 = ( !pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
        y1 = ( pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );
        y2 = ( !pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );
      }
      else
      { // vertical: 2 vehicles behind one another
        m_shape = "2AUVv";

        double delta_x, delta_y;
        bool pos_x = true, pos_y = true; // only for 2nd vehicle
        calcDxDyOperatorsStd(m_inter_vehicle_distance, delta_x, delta_y, pos_x, pos_y);

        x1 = m_follow_center_x;
        x2 = ( pos_x ? x1 + delta_x : x1 - delta_x );
        y1 = m_follow_center_y;
        y2 = ( pos_y ? y1 + delta_y : y1 - delta_y );
      }
      // stringify
      formation_string << x1 << "," << y1 << ":" 
                       << x2 << "," << y2;
      break;
    case 3:
      // 3 AUVs, 3 formations
      double x3, y3;
      if ( m_allowable_width >= 2*m_inter_vehicle_distance )
      { // horizontal
        m_shape = "3AUVh";

        // for the two outside vehicles, calculate offsets
        double delta_x, delta_y;
        bool pos_x1 = true, pos_y1 = true;
        calcDxDyOperators2h(m_inter_vehicle_distance, delta_x, delta_y, pos_x1, pos_y1);

        x1 = ( pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
        x3 = ( !pos_x1 ? m_follow_center_x + delta_x : m_follow_center_x - delta_x );
        y1 = ( pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );
        y3 = ( !pos_y1 ? m_follow_center_y + delta_y : m_follow_center_y - delta_y );
        x2 = m_follow_center_x;
        y2 = m_follow_center_y;
      }
      else if ( m_allowable_width >= m_inter_vehicle_distance )
      { // 1 front, 2 back TODO
        m_shape = "3AUVm";

        x1 = m_follow_center_x;
        y1 = m_follow_center_y;
        x2 = m_follow_center_x - m_inter_vehicle_distance;
        y2 = m_follow_center_y - m_inter_vehicle_distance;
        x3 = m_follow_center_x + m_inter_vehicle_distance;
        y3 = y2;
      }
      else
      { // vertical
        m_shape = "3AUVv";

        double delta_x, delta_y;
        bool pos_x = true, pos_y = true; // only for 2nd vehicle
        calcDxDyOperatorsStd(m_inter_vehicle_distance, delta_x, delta_y, pos_x, pos_y);

        x1 = m_follow_center_x;
        y1 = m_follow_center_y;
        x2 = ( pos_x ? x1 + delta_x : x1 - delta_x );
        y2 = ( pos_y ? y1 + delta_y : y1 - delta_y );
        x3 = ( pos_x ? x1 + 2*delta_x : x1 - 2*delta_x );
        y3 = ( pos_y ? y1 + 2*delta_y : y1 - 2*delta_y );
      }
      // stringify
      formation_string << x1 << "," << y1 << ":" 
                       << x2 << "," << y2 << ":" 
                       << x3 << "," << y3;
      break;
  }
  // notify
  Notify("DESIRED_FORMATION",formation_string.str());
  if ( m_prev_shape != m_shape )
    Notify("FORMATION_SHAPE",m_shape);
}

//---------------------------------------------------------
// Procedure: getDoubleFromNodeReport
//            retrieve any double value from node_report by name
//
double SelectFormation::getDoubleFromNodeReport(std::string full_string, std::string key)
{
  std::string output = getStringFromNodeReport(full_string, key);
  return atof(output.c_str());
}

//---------------------------------------------------------
// Procedure: getStringFromNodeReport
//            retrieve any string value from node_report by name
//
std::string SelectFormation::getStringFromNodeReport(std::string full_string, std::string key)
{
  // example: NAME=anton,X=2676.17,Y=1908.45,SPD=1.48,HDG=316.19,DEP=0,
  //   LAT=34.26380127,LON=-117.17504934,TYPE=SHIP,GROUP=survey,MODE=DRIVE,
  //   ALLSTOP=clear,index=57,YAW=316.19,TIME=1398119728.44,LENGTH=8

  // handle comma-separated string
  std::string output;
  bool valid_msg = true;
  vector<string> svector = parseString(full_string, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    if(param == key)
      output = value;
    else
      valid_msg = false;
  }

  if( output == "" )
    valid_msg = false;

  if(!valid_msg)
    std::cout << GetAppName() << " :: Unhandled NODE_REPORT: " << full_string << std::endl;

  return output;
}

void SelectFormation::calcDxDyOperatorsStd(double const spacing, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y)
{
  double trig_angle;
  double lead_hdg = m_lead_hdg;
  // calculate the angle for trig, given AUV heading
  switch ( quadrant(lead_hdg) )
  {
    case 1:
      trig_angle = lead_hdg;
      pos_x = false;
      pos_y = false;
      break;
    case 2:
      trig_angle = 180-lead_hdg;
      pos_x = false;
      break;
    case 3:
      trig_angle = lead_hdg-180;
      break;
    case 4:
      trig_angle = 360-lead_hdg;
      pos_y = false;
      break;
    default:
      // shouldn't happen
      break;
  }
  // calculate x/y displacement from trig_angle and follow range
  // trig_angle connected edge y, opposite edge x, given above calculations
  // cos trig_angle = dy / range
  // sin trig_angle = dx / range
  delta_x = dx(spacing, trig_angle);
  delta_y = dy(spacing, trig_angle);
}

void SelectFormation::calcDxDyOperators2h(double const spacing, double& delta_x, double& delta_y, bool& pos_x, bool& pos_y)
{
  double trig_angle;
  double lead_hdg = m_lead_hdg;
  switch ( quadrant(lead_hdg) )
  {
    case 1:
      trig_angle = 90-lead_hdg;
      pos_x = false;
      break;
    case 2:
      trig_angle = lead_hdg-90;
      break;
    case 3:
      trig_angle = 270-lead_hdg;
      pos_y = false;
      break;
    case 4:
      trig_angle = lead_hdg-270;
      pos_x = false;
      pos_y = false;
      break;
    default:
      // shouldn't happen
      break;
  }
  delta_x = dx(spacing, trig_angle);
  delta_y = dy(spacing, trig_angle);
}
