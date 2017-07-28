/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SimBioSensor.cpp                                     */
/*    DATE: Jan 25, 2016                                         */
/*                                                               */
/*****************************************************************/

#include "SimBioSensor.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

// read from file
#include <istream>

// atof
#include <stdlib.h>

// handle node report
#include "USCutils.h"

// csv handling
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

// sensor noise
#include <random>
#include <chrono>

//---------------------------------------------------------
// Constructor
//
SimBioSensor::SimBioSensor() :
  m_veh_lon(0.0),
  m_veh_lat(0.0),
  m_veh_depth(-1),
  m_new_lon(false),
  m_new_lat(false),
  m_new_dep(false),
  m_filename(""),
  m_variance(1.0),
  m_output_var(""),
  m_file_read(false),
  m_nav_data_received(false),
  m_lon_step(0.0),
  m_lat_step(0.0),
  m_depth_step(0.0),
  m_verbose(false)
{
  // class variable instantiations can go here
}

//---------------------------------------------------------
// Destructor
//
SimBioSensor::~SimBioSensor()
{
  size_t lon_res = (size_t)d_boundaries_map.at("lon_res");
  size_t lat_res = (size_t)d_boundaries_map.at("lat_res");
  // remove dynamically allocated array
  for ( size_t idlo = 0; idlo < lon_res; idlo++ )
  {
    for ( size_t idla = 0; idla < lat_res; idla++ )
    {
      delete[] d_location_values[idlo][idla];
    }
    delete[] d_location_values[idlo];
  }
  delete[] d_location_values;
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB,
// there is 'new mail', check to see if
// there is anything for this process.
//
bool SimBioSensor::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if ( key == "NAV_LONG" )
    {
      m_veh_lon = dval;
      m_new_lon = true;
    }
    else if ( key == "NAV_LAT" )
    {
      m_veh_lat = dval;
      m_new_lat = true;
    }
    else if ( key == "NAV_DEPTH" )
    {
      m_veh_depth = dval;
      m_new_dep = true;
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;

//    if ( m_nav_data_received == false && !(m_veh_lon == 0 || m_veh_lat == 0 || m_veh_depth == -1) )
//    {
//      m_nav_data_received = true;
//      std::cout << GetAppName() << " :: first nav data received (lon, lat, depth): "
//                << m_veh_lon << ", " << m_veh_lat << ", " << m_veh_depth << std::endl;
//    }
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SimBioSensor::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SimBioSensor::Iterate()
{
  if ( m_new_lon && m_new_lat && m_new_dep )
  {
    m_nav_data_received = true;
    m_new_lon = false;
    m_new_lat = false;
    m_new_dep = false;
  }

  if ( m_file_read && m_nav_data_received )
  {
    // get ground truth
    double dat = getDataPoint();
    if ( m_output_var != "" && dat > 0 )
    {
      // add sensor noise
      dat = addSensorNoise(dat);
      // invert negative values
      dat = (dat < 0 ? -1*dat : dat);
      // publish to MOOSDB
      if ( m_verbose )
        std::cout << GetAppName() << " :: publishing data: " << dat << std::endl;
      m_Comms.Notify(m_output_var, dat);
    }
    m_nav_data_received = false;
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SimBioSensor::OnStartUp()
{
  CMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if ( !m_MissionReader.GetConfiguration(GetAppName(), sParams) )
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();

  STRING_LIST::iterator p;
  for ( p = sParams.begin(); p != sParams.end(); p++ )
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;

    if ( param == "input_filename" )
    {
      m_filename = tolower(value);
      std::cout << GetAppName() << " :: Parameter filename: " << m_filename << std::endl;
      handled = true;
    }
    else if ( param == "sensor_variance" )
    {
      m_variance = atof(value.c_str());
      std::cout << GetAppName() << " :: Parameter sensor_stddev: " << m_stddev << std::endl;
      handled = true;
    }
    else if ( param == "output_var" )
    {
      m_output_var = toupper(value);
      std::cout << GetAppName() << " :: Parameter output_var: " << m_output_var << std::endl;
      handled = true;
    }
    else if ( param == "verbose" )
    {
      m_verbose = (value == "true") ? true : false;
      handled = true;
    }

    if ( !handled )
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
  }

  // read biomass file
  if ( m_filename != "" )
    readBioDataFromFile();

  registerVariables();

  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables
//            at startup, let the MOOSDB know what you want
//            to receive
//
void SimBioSensor::registerVariables()
{
  // get current vehicle position
  m_Comms.Register("NAV_LAT", 0);
  m_Comms.Register("NAV_LONG", 0);
  m_Comms.Register("NAV_DEPTH", 0);
}


// own functions ///////////////////////////////////////////////////////////////

void SimBioSensor::readBioDataFromFile()
{
  // read from file
  // data formatting: tab separated lon, lat, depth, bio_value
  // latitude: north - south
  // longitude: east - west

  std::ifstream input_filestream;
  input_filestream.open(m_filename.c_str(), std::ios::in);

  size_t cnt = 0;

  if ( input_filestream.is_open() )
  {
    std::string line_read;

    // first header line: names
    std::string hdr_line;
    std::getline(input_filestream, hdr_line, '\n');

    // second header line: boundaries
    // (lon_min lon_max lon_res lat_min lat_max lat_res depth_min depth_max depth_res)
    std::getline(input_filestream, line_read, '\n');

    // extract the boundary values given header name and value
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> > tokens(line_read, sep);
    boost::tokenizer< boost::char_separator<char> > hdr_tokens(hdr_line, sep);
    typedef boost::tokenizer< boost::char_separator<char> >::iterator bst_tkn_itr;
    for ( bst_tkn_itr hdr_itr = hdr_tokens.begin(), value_itr = tokens.begin(); hdr_itr != hdr_tokens.end() && value_itr != tokens.end(); hdr_itr++, value_itr++ )
    {
      // add to map
      d_boundaries_map.insert(std::pair<std::string, double>(*hdr_itr,(double)atof((*value_itr).c_str())));
    }

    // check file format
    if ( d_boundaries_map.size() < 9 )
    {
      std::cout << GetAppName() << " :: ERROR: wrong file format, exiting." << std::endl;
      RequestQuit();
    }

    // printout for checking
    std::cout << GetAppName() << " :: \nboundaries stored: " << std::endl;
    std::map<std::string, double>::iterator itr;
    for ( itr = d_boundaries_map.begin(); itr != d_boundaries_map.end(); itr++ )
      std::cout << itr->first << ": " << std::setprecision(10) << itr->second << std::endl;

    // third header line; data labels
    std::getline(input_filestream, line_read, '\n');

    // initialize the array to store values (dynamic allocation, requires destructor)
    size_t lon_res = (size_t)d_boundaries_map.at("lon_res");
    size_t lat_res = (size_t)d_boundaries_map.at("lat_res");
    size_t depth_res = (size_t)d_boundaries_map.at("depth_res");
    std::cout << GetAppName() << " :: resolutions: " << lon_res << "," << lat_res << "," << depth_res << std::endl;
    d_location_values = new double ** [lon_res];
    for ( size_t idlo = 0; idlo < lon_res; idlo++ )
    {
      d_location_values[idlo] = new double*[lat_res];
      for ( size_t idla = 0; idla < lat_res; idla++ )
        d_location_values[idlo][idla] = new double[depth_res];
    }

    // do calculations to figure out indices

    // steps for lon/lat/depth;
    // note there is one less 'step' than the stored resolution,
    // because there is 1 less edge than there are vertices
    m_lon_step = std::abs(d_boundaries_map.at("lon_max") - d_boundaries_map.at("lon_min")) / (lon_res-1);
    m_lat_step = std::abs(d_boundaries_map.at("lat_max") - d_boundaries_map.at("lat_min")) / (lat_res-1);
    m_depth_step = std::abs(d_boundaries_map.at("depth_max") - d_boundaries_map.at("depth_min")) / (depth_res-1);

    // read data
    std::string line;
    while ( std::getline(input_filestream, line) )
    {
      std::string lon, lat, depth, data;

      // nxt: split line, store values
      std::vector<std::string> line_items;
      std::stringstream ss(line);
      std::string item;
      if ( line != "" )
      {
        while ( std::getline(ss, item, ',') )
            line_items.push_back(item);
        // TODO, store by checking header lines for positions
        lon = line_items[0];
        lat = line_items[1];
        depth = line_items[2];
        data = line_items[3];

        cnt++;

        size_t lon_idx, lat_idx, dep_idx;
        double tmp_lon = (double)atof(lon.c_str());
        double tmp_lat = (double)atof(lat.c_str());
        double tmp_dep = (double)atof(depth.c_str());
        lon_idx = round( ( tmp_lon - d_boundaries_map.at("lon_min")) / m_lon_step );
        lat_idx = round( ( tmp_lat - d_boundaries_map.at("lat_min")) / m_lat_step );
        dep_idx = round( ( tmp_dep - d_boundaries_map.at("depth_min")) / m_depth_step );

        d_location_values[lon_idx][lat_idx][dep_idx] = (double)atof(data.c_str());
      }
    }

    // done with this file, close
    input_filestream.close();
  }
  else
    std::cout << GetAppName() << " :: ERROR reading file: " << m_filename << std::endl;

  if ( cnt > 0 )
  {
    std::cout << GetAppName() << " :: Done reading files, objects: " << cnt << '\n';
    m_file_read = true;
  }
}

double SimBioSensor::getDataPoint()
{
  // get boundary values local because we use them twice
  double lon_min = d_boundaries_map.at("lon_min");
  double lon_max = d_boundaries_map.at("lon_max");
  double lat_min = d_boundaries_map.at("lat_min");
  double lat_max = d_boundaries_map.at("lat_max");
  double dep_min = std::abs(d_boundaries_map.at("depth_min"));
  double dep_max = std::abs(d_boundaries_map.at("depth_max"));

  // boundary conditions; lat/lon buffer, buffer = res/3 (third of lane)
  double buffer_lon = std::abs(m_lon_step / 3.0);
  double buffer_lat = std::abs(m_lat_step / 3.0);
  double buffer_depth = std::abs(m_depth_step / 3.0 );

  if ( m_veh_lon >= (lon_min-buffer_lon) && m_veh_lon <= (lon_max+buffer_lon) &&
       m_veh_lat >= (lat_min-buffer_lat) && m_veh_lat <= (lat_max+buffer_lat) &&
       m_veh_depth >= (dep_min-buffer_depth) && m_veh_depth <= (dep_max+buffer_depth) )
  {
    size_t nav_lon_idx, nav_lat_idx, nav_dep_idx;

    if ( m_verbose )
      std::cout << '\n' << GetAppName() << " :: vehicle lon/lat/dep: " << m_veh_lon
                << "," << m_veh_lat << "," << m_veh_depth << std::endl;

    nav_lon_idx = round( (m_veh_lon - lon_min) / m_lon_step );
    nav_lat_idx = round( (m_veh_lat - lat_min) / m_lat_step );
    nav_dep_idx = round( (m_veh_depth - dep_min) / m_depth_step );

    if ( m_verbose )
      std::cout << GetAppName() << " :: calculated index: " << nav_lon_idx
                << "," << nav_lat_idx << "," << nav_dep_idx << std::endl;

    return d_location_values[nav_lon_idx][nav_lat_idx][nav_dep_idx];
  }
  else
  {
    if ( m_verbose )
      std::cout << GetAppName() << " :: outside of data zone" << std::endl;
    return -1;
  }
}

double SimBioSensor::addSensorNoise(double value)
{
  // create a seed, different at every time step
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // start a seeded random number generator
  std::default_random_engine generator(seed);
  // create normal distribution with mean 0.0 and std_dev 1.0
  std::normal_distribution<double> distribution(0.0, m_variance);
  // grab a random number from the distribution
  double noise = distribution(generator);

  return (value + noise);
}
