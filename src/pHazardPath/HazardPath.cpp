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

using namespace std;

//---------------------------------------------------------
// Constructor

HazardPath::HazardPath()
{
  // Config variables
  m_number_of_vehicles = 1;

  // State Variables 
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

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    
    if(key == "UHZ_CONFIG_ACK") 
      handleMailSensorConfigAck(sval);

    else 
      reportRunWarning("Unhandled Mail: " + key);
  }

   calculateSurveyArea();
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
      m_lane_width_overlap = atof(value.c_str());
      handled = true;
    }
    else if( (param == "survey_area_location") ) 
    {
      if( value == "left" )
	m_survey_area_location = 0;
      else if( value == "right")
	m_survey_area_location = 1;
      
      handled = true;
    }    
  
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
 
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void HazardPath::registerVariables()
{
  m_Comms.Register("UHZ_CONFIG_ACK", 0);
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
  double total_box_x = ( ( m_coordinate_4x - m_coordinate_1x ) / 2 ) + m_coordinate_1x ;
  double total_box_y = ( ( m_coordinate_2y - m_coordinate_1y ) / 2 ) + m_coordinate_1y ;
  double total_box_width = fabs( ( m_coordinate_4x - m_coordinate_1x ) / 2 );
  
  if( !m_survey_area_location )
  {
    m_survey_area_x = m_coordinate_1x + (total_box_width/2);
  }
  else
  {
    m_survey_area_x = ( total_box_x ) + ( total_box_width / 2 );
  }
  
  m_survey_area_y =  total_box_y;
  m_survey_area_width = fabs(total_box_width);
  m_survey_area_height = fabs( m_coordinate_2y - m_coordinate_1y );
  m_survey_lane_width = m_swath_width_granted * 2 - m_lane_width_overlap;
  
  cout << m_survey_area_x << "," << m_survey_area_y << "," << m_survey_area_width << ","<< m_survey_lane_width<<endl;
  postWaypointUpdate();
} 

void HazardPath::postWaypointUpdate()
{
  string request = "points=" ;
  
  request += "format=lawnmower" ;
  request += ",label=collab_search";
  
  request += ",x=" + doubleToStringX(m_survey_area_x,2);
  request += ",y=" + doubleToStringX(m_survey_area_y,2);
  request += ",width=" + doubleToStringX(m_survey_area_width,2);
  request += ",height=" + doubleToStringX(m_survey_area_height,2);
  request += ",lane_width=" + doubleToStringX(m_survey_lane_width,2);
  request += ",rows=north-south" ;
  request += ",degs=0";
  
  Notify("WAYPOINT_UPDATES", request);
}
