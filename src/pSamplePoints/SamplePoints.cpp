/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SamplePoints.cpp                                     */
/*    DATE: Jan 25, 2016                                         */
/*                                                               */
/*    This process takes a lawnmower configuration (x,y,meters)  */
/*    and calculates the grid points for sample locations        */
/*                                                               */
/*****************************************************************/

#include "SamplePoints.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

//---------------------------------------------------------
// Constructor
//
SamplePoints::SamplePoints()
{
  // class variable instantiations can go here
  m_output_var = "SAMPLE_POINTS";
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool SamplePoints::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  for ( p=NewMail.begin(); p!=NewMail.end(); p++ ) {
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
    
    if ( key == "TEMPLATE_VAR_IN" )
    {
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SamplePoints::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SamplePoints::Iterate()
{
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SamplePoints::OnStartUp()
{
  CMOOSApp::OnStartUp();
  
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if ( !m_MissionReader.GetConfiguration(GetAppName(), sParams) )
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();

  initGeodesy();

  STRING_LIST::iterator p;
  for ( p=sParams.begin(); p!=sParams.end(); p++ )
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if ( param == "lawnmower_config" )
    {
      // save string .. you might wanna check for format or something
      std::string config = value;
      // TODO parse
      // example:
      // format=lawnmower,label=east-west-survey,x=700,y=1100,width=400,...
      // height=200,lane_width=20,degs=0,startx=0,starty=0,rows=north-south

      // handle comma-separated string
      std::vector<std::string> str_vec = parseString(value, ',');
      unsigned int idx, vec_size = str_vec.size();
      double init = 0;
      double lawn_x = init, lawn_y = init, lawn_width = init, lawn_height = init, lawn_lane_width = init;

      // read the values from the comma-separated string
      for ( idx = 0; idx < vec_size; idx++ ) {
        std::string param = biteStringX(str_vec[idx], '=');
        std::string value = str_vec[idx];
        double dval = (double)atof(value.c_str());

        if ( param == "format" )
        {
          if ( value != "lawnmower" )
          {
            std::cout << GetAppName() << " :: ERROR: wrong format lawnmower config, exiting." << std::endl;
            RequestQuit();
          }
        }
        else if ( param == "x" )
          lawn_x = dval;
        else if ( param == "y" )
          lawn_y = dval;
        else if ( param == "width" )
          lawn_width = dval;
        else if ( param == "height" )
          lawn_height = dval;
        else if ( param == "lane_width" )
          lawn_lane_width = dval;
        // TODO add in degrees for offset
      }
      // check that all data extracted correctly
      if ( lawn_x + lawn_y + lawn_width + lawn_height + lawn_lane_width == 0 )
      {
        std::cout << GetAppName() << " :: Error extracting values from lawnmower config, exiting." << std::endl;
        RequestQuit();
      }
      else
      {
        calculateGridPoints(lawn_x, lawn_y, lawn_width, lawn_height, lawn_lane_width);
      }

      handled = true;
    }
    else if ( param == "output_var" )
    {
      m_output_var = toupper(value);
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
void SamplePoints::registerVariables()
{
//  m_Comms.Register("INPUT_VAR", 0);
}

//---------------------------------------------------------
// Procedure: initGeodesy
//            initialize MOOS Geodesy for lat/lon conversions
//
void SamplePoints::initGeodesy()
{
  // get lat/lon origin from MOOS file
  bool failed = false;
  double latOrigin, longOrigin;
  if ( !m_MissionReader.GetValue("LatOrigin", latOrigin) )
  {
    std::cout << GetAppName() << " :: LatOrigin not set in *.moos file." << std::endl;
    failed = true;
  }
  else if ( !m_MissionReader.GetValue("LongOrigin", longOrigin) )
  {
    std::cout << GetAppName() << " :: LongOrigin not set in *.moos file" << std::endl;
    failed = true;
  }
  else {
    // initialize m_geodesy
    if ( !m_geodesy.Initialise(latOrigin, longOrigin) )
    {
      std::cout << GetAppName() << " :: Geodesy init failed." << std::endl;
      failed = true;
    }
  }
  if ( failed )
  {
    std::cout << GetAppName() << " :: failed to initialize geodesy, exiting." << std::endl;
    RequestQuit();
  }
}

//---------------------------------------------------------
// Procedure: calculateGridPoints
//            calculate grid points for given
//
void SamplePoints::calculateGridPoints(double ctr_x, double ctr_y, double width, double height, double lane_width)
{
  // determine where the southwest corner (0,0) point is
  double lawn_sw_corner_x = ctr_x - width/2.0;
  double lawn_sw_corner_y = ctr_y - height/2.0;

  // convert corner to lat/lon
  double sw_corner_lon = 0;
  double sw_corner_lat = 0;
  m_geodesy.UTM2LatLong(lawn_sw_corner_x, lawn_sw_corner_y, sw_corner_lat, sw_corner_lon);

  // convert lane width to lat/lon
  // length of deg lat/lon via http://www.csgnetwork.com/degreelenllavcalc.html at 34.09
  // 1 deg lat in m: 110923.99118801417m
  // 1 deg lon in m: 92287.20804979937m
  double lat_deg_to_m = 110923.99118801417;
  double lon_deg_to_m = 92287.20804979937;
  double lw_lon = lane_width*lon_deg_to_m; // convert lane width to lon
  double lw_lat = lane_width*lat_deg_to_m; // convert lane width to lat

  // checking
  std::cout << "SW corner: " << sw_corner_lon << ", " << sw_corner_lat << std::endl;
  std::cout << "lane_width lon, lat: " << lw_lon << ", " << lw_lat << std::endl;

  // calculate nr lanes
  size_t lanes_x = std::floor(width / lane_width);
  size_t lanes_y = std::floor(height / lane_width);
  std::cout << "lanes (x, y): " << lanes_x << ", " << lanes_y << std::endl;

  // calculate grid pts
  std::vector< std::pair<double,double> > grid_pts;
  for ( int d_x = 0; d_x < lanes_x; d_x++ )
  {
    for ( int d_y = 0; d_y < lanes_y; d_y++ )
    {
      grid_pts.push_back(std::pair<double,double>(sw_corner_lon + d_x * lw_lon, sw_corner_lat + d_y * lw_lat));
    }
  }

  publishGridPoints(grid_pts);

  std::cout << GetAppName() << " :: Number of grid points: " << grid_pts.size();
}

//---------------------------------------------------------
// Procedure: publishGridPoints
//
void SamplePoints::publishGridPoints(std::vector< std::pair<double, double> > grid_vector)
{
  // serialize
  std::vector< std::pair<double, double> >::iterator grid_vec_itr;
  std::ostringstream output_ss;
  for ( grid_vec_itr = grid_vector.begin(); grid_vec_itr != grid_vector.end(); grid_vec_itr++ )
  {
    std::pair<double, double> grid_pt = *grid_vec_itr;
    output_ss << grid_pt.first << "," << grid_pt.second << ";";
  }
  std::cout << "output string: " << output_ss.str() << std::endl;

  m_Comms.Notify(m_output_var, output_ss.str());
}
