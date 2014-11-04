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
  debug = false;
  
  m_sensor_range = 0;
  m_sensor_width = 0;
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
    
    if ( key == "NODE_REPORT_LOCAL" )
    { // _LOCAl = ownship info
        double lead_x, lead_y, lead_hdg;
        // need contact thingy to extract position of any name from node report
        lead_x = getDoubleFromNodeReport(sval, "X");
        lead_y = getDoubleFromNodeReport(sval, "Y");
        lead_hdg = getDoubleFromNodeReport(sval, "HDG");
        if (debug)
          std::cout << "vehicle: " << lead_x << "," << lead_y << "," << lead_hdg << std::endl;

        // evaluate the formation width for updated position
        double max_form_width = 0;
        checkUpcomingLakeOutline(lead_x, lead_y, lead_hdg, max_form_width);
        std::cout << "Publishing allowable width: " << max_form_width << std::endl;
        Notify("ALLOWABLE_WIDTH_FORM", max_form_width);
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
    if ( (param == "sensor_range") && isNumber(value) )
    {
      // assuming the atof works, store the val
      m_sensor_range = atof(value.c_str());
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
    else if ( param == "sensor_width" && isNumber(value) )
      m_sensor_width = atof(value.c_str());

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
  calcDxDyOperatorsStd(m_sensor_range, lead_hdg, delta_x, delta_y, pos_x, pos_y);
  // need to m_follow_range ahead, given hdg
  double horizon_x = 0.0, horizon_y = 0.0;
  horizon_x = ( !pos_x ? lead_x + delta_x : lead_x - delta_x );
  horizon_y = ( !pos_y ? lead_y + delta_y : lead_y - delta_y );
  // visualize
  std::ostringstream hor_ctr;
  hor_ctr << "x=" << horizon_x << ",y=" << horizon_y << ",label=horizon_ctr";
  Notify("VIEW_POINT",hor_ctr.str());

  // construct a line at sensor_range
  // old: m_num_vehicles*m_ivd; // num_veh*ivd length, at time_horizon
  double line_length = m_sensor_width; 
  // propagate current position to time_horizon
  bool pos_x1 = true, pos_y1 = true;
  calcDxDyOperators2h(line_length/2.0, lead_hdg, delta_x, delta_y, pos_x1, pos_y1);
  double line_1x = (pos_x1 ? horizon_x + delta_x : horizon_x - delta_x );
  double line_1y = (pos_y1 ? horizon_y + delta_y : horizon_y - delta_y );
  double line_2x = (!pos_x1 ? horizon_x + delta_x : horizon_x - delta_x );
  double line_2y = (!pos_y1 ? horizon_y + delta_y : horizon_y - delta_y );
  // visualize
  if (debug)
  {
    std::ostringstream pt1;
    pt1 << "x=" << line_1x << ",y=" << line_1y << ",label=h1";
    Notify("VIEW_POINT",pt1.str());
    std::ostringstream pt2;
    pt2 << "x=" << line_2x << ",y=" << line_2y << ",label=h2";
    Notify("VIEW_POINT",pt2.str());
  }
  std::ostringstream segl;
  segl << "pts={" << line_1x << "," << line_1y << ":" << line_2x << "," << line_2y 
       << "},label=sensor_horizon,edge_color=darkseagreen,vertex_color=darkseagreen,vertex_size=1,edge_size=1";
  Notify("VIEW_SEGLIST",segl.str());

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
  
  if ( output.size() > 0 )
  {
    std::vector<line_str>::iterator line_iter;
    max_form_width = 0;
    double overallMin = std::numeric_limits<double>::max();
    for ( line_iter = output.begin(); line_iter != output.end(); line_iter++ )
    {
      // get endpoints linestring (linestring is a vector)
      double x1, y1, x2, y2;
      x1 = (*line_iter).at(0).x();
      x2 = (*line_iter).at(1).x();
      y1 = (*line_iter).at(0).y();
      y2 = (*line_iter).at(1).y();
      if (debug)
      {
        std::cout << "check size line string vector: " << (*line_iter).size() << std::endl; // should be 2
        // line string elements are model::d2::point_xy
        std::cout << "first element: " << x1 << "," << y1 << std::endl;
        std::cout << "second element: " << x2 << "," << y2 << std::endl;
      }
      // temp, show them, so I can see what's created
      std::ostringstream segl;
      segl << "pts={" << x1 << "," << y1 << ":" << x2 << "," << y2
           << "},label=intersection,edge_color=white,vertex_color=red,vertex_size=2,edge_size=1";
      Notify("VIEW_SEGLIST",segl.str());
      
      // only calculate reduced width if the line segment
      // contains the horizon center point, else, it's a little piece 
      // off-center, so we return min width (do not set max_form_width)
      // note: should we return biggest piece as well as the center of it? -> shift formation
      line_str isect_linestr;
      // using boost assign:
      isect_linestr += line_pt(x1, y1), line_pt(x2, y2);
      line_pt hor_ctr;
      hor_ctr = line_pt(horizon_x,horizon_y);
      double dist = boost::geometry::distance(hor_ctr, isect_linestr);
      if ( dist < 0.001) // it should be 0
      {
        // calculate distance endpoints to line center
        double euclidDistance1, euclidDistance2;
        euclidDistance(x1, y1, horizon_x, horizon_y, euclidDistance1);
        euclidDistance(x2, y2, horizon_x, horizon_y, euclidDistance2);
        // take min distance endpts to line center as half formation width
        double minDist = min(euclidDistance1, euclidDistance2)*2.0;
        if ( minDist < overallMin )
          overallMin = minDist;
      }
    }
    if ( overallMin < std::numeric_limits<double>::max() )
      max_form_width = overallMin;
  }
  else
  {
    // no intersection line segments
    std::cout << "no intersecting line segments" << std::endl;
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
