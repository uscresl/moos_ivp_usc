/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: MaxFormationWidth.cpp                                */
/*    DATE: Apr 24, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#include "MaxFormationWidth.h"

#include <iterator>
#include "math.h"
#include <limits>
#include <algorithm>

// boost includes in header file
#include <boost/assign.hpp>
using namespace boost::assign;

using namespace std;

//---------------------------------------------------------
// Constructor
//
MaxFormationWidth::MaxFormationWidth()
{
  // class variable instantiations can go here
  debug = true;
  
  m_ivd = 0;
  m_time_horizon = 0;
  m_lead_vehicle = "anton";

  m_num_vehicles = 1;
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool MaxFormationWidth::OnNewMail(MOOSMSG_LIST &NewMail)
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
    
    if ( key == "NUM_VEHICLES" )
      m_num_vehicles = (size_t)round(dval);
    else if ( key == "NODE_REPORT_LOCAL" )
    {
//      // check if right vehicle
//      std::string veh_name = getStringFromNodeReport(sval, "NAME");
//      if ( veh_name == m_lead_vehicle )
//      {
        double lead_x, lead_y, lead_hdg;
        // need contact thingy to extract position of any name from node report
        lead_x = getDoubleFromNodeReport(sval, "X");
        lead_y = getDoubleFromNodeReport(sval, "Y");
        lead_hdg = getDoubleFromNodeReport(sval, "HDG");
        if (debug)
          std::cout << "vehicle: " << lead_x << "," << lead_y << "," << lead_hdg << std::endl;

        // evaluate the formation width for updated position
        double max_form_width;
        checkUpcomingLakeOutline(lead_x, lead_y, lead_hdg, max_form_width);
        Notify("ALLOWABLE_WIDTH_FORM", max_form_width);
//      }
    }
//    else
//      std::cout << "pMaxFormationWidth :: Unhandled Mail: " << key << std::endl;
      //reportRunWarning("Unhandled Mail: " + key);
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool MaxFormationWidth::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool MaxFormationWidth::Iterate()
{
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool MaxFormationWidth::OnStartUp()
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
    if ( (param == "time_horizon") && isNumber(value) )
    {
      // assuming the atof works, store the val
      m_time_horizon = atof(value.c_str());
      handled = true;
    }
    else if( (param == "lake_outline") )
    {
      // this should be a 'x1,y1:x2,y2:..:xn,yn' string with the polygon
      publishToView(value);
      convertToWKT(value);
      GeometryFromWKT(value, m_lake_outline);
      if ( debug )
        std::cout << "\n Testing polygon, show nr pts: " << boost::geometry::num_points(m_lake_outline) << std::endl;
      handled = true;
    }
    else if ( param == "lead_vehicle_name" )
      m_lead_vehicle = value;
    else if ( (param == "inter_vehicle_distance") && isNumber(value) )
    {
      // assuming the atof works, store the val
      m_ivd = atof(value.c_str());
      handled = true;
    }

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
void MaxFormationWidth::registerVariables()
{
  m_Comms.Register("NODE_REPORT_LOCAL", 0);
  m_Comms.Register("NUM_VEHICLES", 0);
}

void MaxFormationWidth::convertToWKT(std::string & str)
{
  // example input: 30,10:40,40:20,40:10,20:30,10
  // example WKT: POLYGON ((30 10, 40 40, 20 40, 10 20, 30 10))
  // handle comma-separated string
  // replace from algorithm header
  std::replace(str.begin(), str.end(),',',' ');
  std::replace(str.begin(), str.end(),':',',');
  std::ostringstream out;
  out << "POLYGON((" << str << "))";
  str = out.str();
  if ( debug )
    std::cout << "\n full WKT string: " << str << std::endl;
}

void MaxFormationWidth::GeometryFromWKT(std::string const wkt, MaxFormationWidth::b_polygon & poly)
{
  // convert wkt string to a boost geometry polygon
  boost::geometry::read_wkt(wkt, poly);
}

void MaxFormationWidth::checkUpcomingLakeOutline(double const lead_x, double const lead_y, double const lead_hdg, double & max_form_width)
{
  // calculate horizon center point
  double delta_x, delta_y;
  bool pos_x = true, pos_y = true;
  // calculate new follow center
  calcDxDyOperatorsStd(m_time_horizon, lead_hdg, delta_x, delta_y, pos_x, pos_y);
  // need to m_follow_range ahead, given hdg
  double horizon_x = 0.0, horizon_y = 0.0;
  horizon_x = ( !pos_x ? lead_x + delta_x : lead_x - delta_x );
  horizon_y = ( !pos_y ? lead_y + delta_y : lead_y - delta_y );
  std::ostringstream hor_ctr;
  hor_ctr << "x=" << horizon_x << ",y=" << horizon_y << ",label=horizon_ctr";
  Notify("VIEW_POINT",hor_ctr.str());

  // construct a line, num_veh*ivd length, at time_horizon
  double line_length = m_num_vehicles*m_ivd;
  // propagate current position to time_horizon
  bool pos_x1 = true, pos_y1 = true;
  calcDxDyOperators2h(line_length/2.0, lead_hdg, delta_x, delta_y, pos_x1, pos_y1);
  double line_1x = (pos_x1 ? horizon_x + delta_x : horizon_x - delta_x );
  double line_1y = (pos_y1 ? horizon_y + delta_y : horizon_y - delta_y );
  double line_2x = (!pos_x1 ? horizon_x + delta_x : horizon_x - delta_x );
  double line_2y = (!pos_y1 ? horizon_y + delta_y : horizon_y - delta_y );
  if (debug)
    std::cout << "line points: " << line_1x << "," << line_1y << ":" << line_2x << "," << line_2y << std::endl;
  std::ostringstream pt1;
  pt1 << "x=" << line_1x << ",y=" << line_1y << ",label=l1";
  Notify("VIEW_POINT",pt1.str());
  std::ostringstream pt2;
  pt2 << "x=" << line_2x << ",y=" << line_2y << ",label=l2";
  Notify("VIEW_POINT",pt2.str());

  // make boost geometry linestring
  // eg. LINESTRING (30 10, 10 30, 40 40)
  typedef boost::geometry::model::d2::point_xy<double> line_pt;
  boost::geometry::model::linestring<line_pt> b_line_str;
  // using boost assign:
  b_line_str += line_pt(line_1x, line_1y), line_pt(line_2x, line_2y);
  if (debug)
    std::cout << "\n check linestring: length: " << boost::geometry::length(b_line_str) << std::endl;

  // check intersection boost geometries: 
  // polygon + linestring -> multiple linestrings
  typedef boost::geometry::model::linestring<line_pt> line_str;
  std::vector<line_str> output;
  boost::geometry::intersection(m_lake_outline, b_line_str, output);
  if (debug)
  {
    std::cout << "\n length vector returned: " << output.size() << std::endl;
    if (output.size() > 0)
      std::cout << "\n length of first linestring returned: " << boost::geometry::length(output[0]) << std::endl;
  }
  
  // set the max formation width to max width returned
  // typically, we will only get 1 width returned anyway, 
  // unless the vehicle were to head straight at a pier or something
  // nb. we are not yet accounting for the fact that this measure can be skewed
  //     as per the boat heading (eg. only space on one side of boat)
  std::vector<line_str>::iterator line_iter;
  max_form_width = 0;
  for ( line_iter = output.begin(); line_iter != output.end(); line_iter++ )
  {
    double length = boost::geometry::length(*line_iter);
    if ( length > max_form_width )
      max_form_width = length;
  }
}

void MaxFormationWidth::publishToView(std::string const str)
{
  // example VIEW_POLYGON
  // pts={2100,1800:2100,2600:3100,2600:3100,1800},label=OpRegion,vertex_size=2,edge_size=2
  std::ostringstream view_str;
  view_str << "pts={" << str << "},label=LakeOutline,edge_color=yellow,vertex_color=yellow,vertex_size=2,edge_size=2";
  Notify("VIEW_SEGLIST",view_str.str());
}
