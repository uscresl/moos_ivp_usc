/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: HazardMgr.cpp                                        */
/*    DATE: Oct 26th 2012                                        */
/*                                                               */
/*    Adapted by:                                                */
/*    NAME: Supreeth Subbaraya, Stephanie Kemna                  */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    DATE: Apr, 2013                                            */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*****************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "HazardPath.h"
#include "XYFormatUtilsHazard.h"
#include "ACTable.h"
#include "math.h"

#include <limits>

using namespace std;

//---------------------------------------------------------
// Constructor

HazardPath::HazardPath()
{
  // Config variables
  m_number_of_vehicles = 1;

  // State Variables 
  m_num_surveys = 1;
  m_surveys_done = 0;
  m_swath_width_granted = 0;

  // survey modes and waypoints
  m_survey_mode = "lawnmower";
  m_first_wpt = true;
  m_previous_wpt = std::pair<double,double>(0,0);
  
  // time check
  m_start_x = std::numeric_limits<double>::min();
  m_start_y = std::numeric_limits<double>::min();
  m_mission_time = 9000;
  m_return_threshold = 120;
  m_speed = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool HazardPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

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
    
    if( key == "UHZ_CONFIG_ACK" ) 
    {
      handleMailSensorConfigAck(sval);
      if (m_survey_mode == "lawnmower")
        calculateSurveyArea();
    }
    else if( (key == "SURVEY_DONE") )
    {
      if ( m_survey_mode == "lawnmower" )
      {
        m_surveys_done++;
        if( sval == "true" && (m_surveys_done == m_num_surveys) )
          Notify("RETURN", "true"); // all requested surveys done, time to return
        else
          postWaypointUpdate();
      }
      else if ( m_survey_mode == "follow" )
        postWaypointFollow();
    }
    else if ( (key == "HAZARD_REPORT") && (m_survey_mode == "follow") )
    {
      std::cout << "got a hazard report" << std::endl;
      handleHazardReport(sval);
    }
    else if ( key == "DB_UPTIME" )
    { // have both vehicles return in time to report
      double dbtime = dval;
      // check whether to return to be back at start pos within mission time
      double distance_to_start = sqrt( pow(m_start_x - m_x, 2) + pow(m_start_y - m_y, 2) );
      double time_to_return = distance_to_start / m_speed;
      if ( (m_mission_time - dbtime - time_to_return) <= m_return_threshold )
        Notify("RETURN","true");
    }
    else if ( key == "NAV_X" )
    {
      m_x = dval;
      if ( m_start_x == std::numeric_limits<double>::min() )
        m_start_x = dval;
    }
    else if ( key == "NAV_Y" )
    {
      m_y = dval;
      if ( m_start_y == std::numeric_limits<double>::min() )
        m_start_y = dval;
    }
    else if ( key == "NAV_SPEED" )
      m_speed = dval;
    else
      reportRunWarning("Unhandled Mail: " + key);
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool HazardPath::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool HazardPath::Iterate()
{
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HazardPath::OnStartUp()
{

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) 
  {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if((param == "number_of_vehicles") && isNumber(value)) 
    {
      m_number_of_vehicles = atof(value.c_str());
      handled = true;
    }
    else if( (param == "coordinate_1") ) 
    {
      string xValue = tolower(biteString(value, ','));
      string yValue = value;
      m_coordinate_1x = atof(xValue.c_str());
      m_coordinate_1y = atof(yValue.c_str());
      m_previous_wpt = std::pair<double,double>(m_coordinate_1x,m_coordinate_1y);
      handled = true;
    }
    else if( (param == "coordinate_2") ) 
    {
      string xValue = tolower(biteStringX(value, ','));
      string yValue = value;
      m_coordinate_2x = atof(xValue.c_str());
      m_coordinate_2y = atof(yValue.c_str());
      handled = true;
    }
    else if( (param == "coordinate_3") ) 
    {
      string xValue = tolower(biteStringX(value, ','));
      string yValue = value;
      m_coordinate_3x = atof(xValue.c_str());
      m_coordinate_3y = atof(yValue.c_str());
      handled = true;
    }
    else if( (param == "coordinate_4") ) 
    {
      string xValue = tolower(biteStringX(value, ','));
      string yValue = value;
      m_coordinate_4x = atof(xValue.c_str());
      m_coordinate_4y = atof(yValue.c_str());
      handled = true;
    }
    else if( (param == "lane_width_overlap") && isNumber(value) ) 
    {
      // how much lanes should overlap
      m_lane_width_overlap = atof(value.c_str());
      handled = true;
    }
    else if( (param == "survey_area_location") ) 
    {
      // which of the survey areas to take, [0,..> index
      m_survey_area_location = atoi(value.c_str());
      handled = true;
    }
    else if ( param == "survey_mode" )
    {
      if ( value == "follow" )
        m_survey_mode = value;
      // default is lawnmower
      std::cout << "survey mode: " << m_survey_mode << std::endl;
    }
    else if ( param == "mission_time" )
    {
      m_mission_time = atof(value.c_str());
      handled = true;
      std::cout << "mission time: " << m_mission_time << std::endl;
    }
    else if ( param == "time_at_end" )
    {
      m_return_threshold = atof(value.c_str());
      handled = true;
      std::cout << "time at end: " << m_return_threshold << std::endl;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // let's show the area on pMarineViewer
  std::string poly = "pts={" + doubleToStringX(m_coordinate_1x,2) + "," 
                             + doubleToStringX(m_coordinate_1y,2) + ":"
                             + doubleToStringX(m_coordinate_2x,2) + "," 
                             + doubleToStringX(m_coordinate_2y,2) + ":"
                             + doubleToStringX(m_coordinate_3x,2) + "," 
                             + doubleToStringX(m_coordinate_3y,2) + ":"
                             + doubleToStringX(m_coordinate_4x,2) + "," 
                             + doubleToStringX(m_coordinate_4y,2)
                             + ",edge_color=yellow,vertex_color=green,vertex_size=3,edge_size=2";
  Notify("VIEW_POLYGON",poly);

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void HazardPath::registerVariables()
{
  m_Comms.Register("UHZ_CONFIG_ACK", 0);
  m_Comms.Register("SURVEY_DONE", 0);
  m_Comms.Register("HAZARD_REPORT", 0);
  m_Comms.Register("DB_UPTIME", 0);
  m_Comms.Register("NAV_X", 0);
  m_Comms.Register("NAV_Y", 0);
  m_Comms.Register("NAV_SPEED", 0);
}

//---------------------------------------------------------
// Procedure: handleMailSensorConfigAck

bool HazardPath::handleMailSensorConfigAck(string str)
{
  // Expected ack parameters:
  string vname, width, pd, pfa, pclass;
  
  // Parse and handle ack message components
  bool   valid_msg = true;
  string original_msg = str;

  vector<string> svector = parseString(str, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];

    if(param == "vname")
      vname = value;
    else if(param == "pd")
      pd = value;
    else if(param == "width")
      width = value;
    else if(param == "pfa")
      pfa = value;
    else if(param == "pclass")
      pclass = value;
    else
      valid_msg = false;       
  }

  if((vname=="")||(width=="")||(pd=="")||(pfa=="")||(pclass==""))
    valid_msg = false;
  
  if(!valid_msg)
    reportRunWarning("Unhandled Sensor Config Ack:" + original_msg);

  if(valid_msg) 
  {
    m_swath_width_granted = atof(width.c_str());
    m_pd_granted = atof(pd.c_str());
  }

  return(valid_msg);
}

void HazardPath::calculateSurveyArea()
{
  // full survey box
  double total_box_x = ( ( m_coordinate_4x - m_coordinate_1x ) / 2 ) + m_coordinate_1x ;
  double total_box_y = ( ( m_coordinate_2y - m_coordinate_1y ) / 2 ) + m_coordinate_1y ;
  double total_box_width = fabs( m_coordinate_4x - m_coordinate_1x );
  
  // calculate specific box for each vehicle
  m_survey_area_width = fabs(total_box_width) / m_number_of_vehicles;
  m_survey_area_x = m_coordinate_1x 
                    + ((m_survey_area_location+1) * (m_survey_area_width))
                    - (m_survey_area_width/2);

  if( m_survey_area_x < 0 )
    m_survey_order = ( (m_survey_area_location % 2) == 0 ? 0 : 1);
  else
    m_survey_order = ( (m_survey_area_location % 2) == 0 ? 1 : 0);

  m_survey_area_y =  total_box_y;
  m_survey_area_height = fabs( m_coordinate_2y - m_coordinate_1y );
  m_survey_lane_width = m_swath_width_granted * 2 - m_lane_width_overlap;
  
  switch((int)round(m_swath_width_granted))
  {
    // based on the knowledge we have for time it takes to survey area, and
    // 9000sec time limit on full survey
    case 5:
      m_num_surveys = 1;
      break;
    case 10:
      m_num_surveys = 2;
      break;
    case 25:
      m_num_surveys = 4;
      break;
    case 50:
      m_num_surveys = 8;
      break;
    default:
      m_num_surveys = 1;
  }
  
  postWaypointUpdate();
}

void HazardPath::postWaypointUpdate()
{
  string request = "points=" ;
  request += "format=lawnmower" ;
  request += ",label=collab_search";
  request += ",x=" + doubleToStringX(m_survey_area_x,2);
  request += ",y=" + doubleToStringX(m_survey_area_y,2);
  // adding lead for survey area to avoid turning in survey area
  double survey_height = m_survey_area_height;
  double survey_width = m_survey_area_width;
  ( (m_surveys_done % 2 == 0) ? survey_height += 32 : survey_width += 32 );
  request += ",width=" + doubleToStringX(survey_width,2);
  request += ",height=" + doubleToStringX(survey_height,2);
  request += ",lane_width=" + doubleToStringX(m_survey_lane_width,2);
  request =  request + ",rows=" + ( (m_surveys_done % 2 == 0) ? "north-south" : "east-west" ) ;
  request += ",degs=0";
  request =  request + "#order=" + (( m_survey_order ) ? "normal" : "reverse");
  
  Notify("WAYPOINT_UPDATES", request);
}

void HazardPath::handleHazardReport(std::string sval)
{
  // HAZARD_REPORT = x=-14.2,y=-293.6,label=08,type=hazard
  string xString, yString, typeString;
  bool valid_msg = true;
  vector<string> svector = parseString(sval, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++)
  {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    if(param == "x")
      xString = value;
    else if(param == "y")
      yString = value;
    else if(param == "type")
      typeString = value;
    else
      valid_msg = false;
  }
  // push back
  // currently; only inspect objects classified as hazard
  if ( typeString == "hazard" )
  {
    m_survey_waypoints.push_back( std::pair<double,double>(atof(xString.c_str()),atof(yString.c_str())) );
    std::cout << "pushed back: " << xString << "," << yString << std::endl;
    if ( m_first_wpt )
    {
      postWaypointFollow();
      m_first_wpt = false;
    }
  }
}

void HazardPath::postWaypointFollow()
{
  bool north_south = true;
  if ( m_survey_waypoints.size() > 0 )
  { // take a new wpt if possible
    m_previous_wpt = m_survey_waypoints.front();
    m_survey_waypoints.pop_front();
    m_survey_waypoints_second.push_back(m_previous_wpt);
  }
  else if ( m_survey_waypoints_second.size() > 0 )
  { // if no new wpt, resurvey previous ones in orthogonal orientation
    m_previous_wpt = m_survey_waypoints_second.front();
    m_survey_waypoints_second.pop_front();
    north_south = false;
  }
  // else, if still nothing new, survey previous point again. 
  // TODO: we could do something smarter here, like random exploration?

  double xval = m_previous_wpt.first;
  double yval = m_previous_wpt.second;

  // go straight over the hazard; these simulated vehicles don't suffer from
  // not being able to sense right underneath themselves
  double xMin, xMax, yMin, yMax;
  if ( north_south )
  {
    xMin = xval; 
    xMax = xval + (m_swath_width_granted*2);
    yMin = yval - 20; // turning takes up 16y
    yMax = yval + 20;
  }
  else
  {
    xMin = xval - 20; // turning takes up 16y
    xMax = xval + 20;
    yMin = yval; 
    yMax = yval + (m_swath_width_granted*2);
  }

  std::string update;
  update = "points=" + doubleToStringX(xMin) + "," + doubleToStringX(yMin) + ":" 
                     + doubleToStringX(xMin) + "," + doubleToStringX(yMax) + ":"
                     + doubleToStringX(xMax) + "," + doubleToStringX(yMax) + ":"
                     + doubleToStringX(xMax) + "," + doubleToStringX(yMin);
  Notify("WAYPOINT_UPDATES", update);
}
