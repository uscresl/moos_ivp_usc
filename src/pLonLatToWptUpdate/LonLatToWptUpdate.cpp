/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: LonLatToWptUpdate.cpp                                */
/*    DATE: Feb 5, 2016                                          */
/*                                                               */
/*****************************************************************/

#include "LonLatToWptUpdate.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

// string tokenizer
#include <boost/tokenizer.hpp>

//---------------------------------------------------------
// Constructor
//
LonLatToWptUpdate::LonLatToWptUpdate()
{
  // class variable instantiations can go here
  m_input_var_lonlat = "";
  m_output_var_wpt_update = "";
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB,
// there is 'new mail', check to see if
// there is anything for this process.
//
bool LonLatToWptUpdate::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if ( key == m_input_var_lonlat )
      handleMailLonLatToWptUpdateVarIn(sval);
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool LonLatToWptUpdate::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool LonLatToWptUpdate::Iterate()
{
  // nothing here, we only run when we receive input,
  // and handle it straight away
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool LonLatToWptUpdate::OnStartUp()
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

    bool handled = true;
    if ( (param == "input_var_lonlat") )
      m_input_var_lonlat = toupper(value);
    else if( param == "output_var_wpt_update" )
      m_output_var_wpt_update = toupper(value);
    else
      handled = false;

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
void LonLatToWptUpdate::registerVariables()
{
  if ( m_input_var_lonlat != "" )
    m_Comms.Register(m_input_var_lonlat, 0);
}

//---------------------------------------------------------
// Procedure: initGeodesy
//            initialize MOOS Geodesy for lat/lon conversions
//
void LonLatToWptUpdate::initGeodesy()
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
  else
    std::cout << GetAppName() << " :: Geodesy initialized: lonOrigin, latOrigin = " << longOrigin << ", " << latOrigin << std::endl;
}

//---------------------------------------------------------
// Procedure: handleMailLonLatToWptUpdateVarIn
//            a place to do more advanced handling of the
//            incoming message
//
void LonLatToWptUpdate::handleMailLonLatToWptUpdateVarIn(std::string str)
{
  std::cout << GetAppName() << " :: processing mail: " << str << std::endl;

  std::ostringstream converted_str;

  boost::char_separator<char> ch_sep(":");
  boost::tokenizer<boost::char_separator<char>> all_waypoints(str, ch_sep);
  for ( const auto & waypoint : all_waypoints )
  {
    // convert lon/lat to x/y

    // expected format string: "<lon>,<lat>"
    // handle comma-separated string
    std::vector<std::string> svector = parseString(waypoint, ',');
    double lon = atof(svector[0].c_str());
    double lat = atof(svector[1].c_str());

    // convert to x/y
    double lx, ly;
    bool converted = lonLatToUTM(lon, lat, lx, ly);

    if ( converted )
      converted_str << lx << "," << ly << ":";
  }

  if ( converted_str.str() != "" )
    publishWpts(converted_str.str());
  else
    std::cout << GetAppName() << " :: ERROR converting waypoints: " << str << std::endl;
}

//---------------------------------------------------------
// Procedure: publishWpts
//            format and publish the x/y values for wpt bhv update
//
void LonLatToWptUpdate::publishWpts(std::string list_of_waypoints)
{
  std::ostringstream output;
  output << "points=" << list_of_waypoints;
  std::string output_str = output.str();
  std::cout << GetAppName() << " :: Publishing: " << output_str << std::endl;
  Notify(m_output_var_wpt_update, output_str);
}

//---------------------------------------------------------
// Procedure: lonLatToUTM
//            use MOOS Geodesy to convert lon/lat to x/y
//
bool LonLatToWptUpdate::lonLatToUTM (double lon, double lat, double & lx, double & ly )
{
  bool successful = m_geodesy.LatLong2LocalUTM(lat, lon, ly, lx);

  if ( !successful )
    std::cout << GetAppName() << " :: ERROR converting lon/lat to x/y.\n";

  return successful;
}
