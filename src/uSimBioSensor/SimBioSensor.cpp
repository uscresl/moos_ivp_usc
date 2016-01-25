/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SimBioSensor.cpp                                     */
/*    DATE: Nov 19, 2015                                         */
/*                                                               */
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

//---------------------------------------------------------
// Constructor
//
SimBioSensor::SimBioSensor()
{
  // class variable instantiations can go here

  // MOOS variables
  m_veh_lon = 0;
  m_veh_lat = 0;
  m_veh_depth = 0;

  // params
  m_filename = "";
  m_output_var = "";

  // class vars
  m_file_read = false;
  m_nav_data_received = false;
}

//---------------------------------------------------------
// Destructor
//
SimBioSensor::~SimBioSensor()
{
  size_t lon_res = (size_t)d_boundaries_map.at("lon_res");
  size_t lat_res = (size_t)d_boundaries_map.at("lat_res");
  // remove dynamically allocated array
  for ( int idlo = 0; idlo < lon_res+1; idlo++ )
  {
    for ( int idla = 0; idla < lat_res+1; idla++ )
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
    
    if ( key == "NODE_REPORT_LOCAL" )
    {
      //handle
      m_veh_lon = getDoubleFromNodeReport(sval,"LON");
      m_veh_lat = getDoubleFromNodeReport(sval,"LAT");
      m_veh_depth = getDoubleFromNodeReport(sval,"DEP");

      if ( !m_nav_data_received )
        m_nav_data_received = true;
    }
    else
      std::cout << "uSimBioSensor :: Unhandled Mail: " << key << std::endl;
      //reportRunWarning("Unhandled Mail: " + key);
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
  //std::cout << "iterate, proceed? " << (m_file_read && m_nav_data_received) << std::endl;
  if ( m_file_read && m_nav_data_received )
  {
//    findClosestDataPoint();
    double dat = getDataPoint();
    if ( m_output_var != "" && dat > 0 )
    {
      std::cout << "publishing data: " << dat << std::endl;
      m_Comms.Notify(m_output_var, dat);
    }
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
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();
    //reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) 
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;

    if ( param == "filename" )
    {
      m_filename = tolower(value);
      std::cout << GetAppName() << " :: Parameter filename: " << m_filename << std::endl;
      handled = true;
    }
    else if ( param == "output_var" )
    {
      m_output_var = toupper(value);
      std::cout << GetAppName() << " :: Parameter output_var: " << m_output_var << std::endl;
    }

    if(!handled)
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
      //reportUnhandledConfigWarning(orig);
  }

  // read biomass file
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
  // because we want lon/lat/depth at the same time, we might as well use
  // the node report
  m_Comms.Register("NODE_REPORT_LOCAL", 0);
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

  std::string line_read;
  std::istringstream line_stream;
  size_t cnt = 0;
  if ( input_filestream.is_open() )
  {
    // first header line: names
    std::string hdr_line;
    std::getline(input_filestream, hdr_line);

    // second header line: boundaries
    // (lon_min lon_max lon_res lat_min lat_max lat_res depth_min depth_max depth_res)
    std::getline(input_filestream, line_read);

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
    // printout for checking
    std::cout << "\nboundaries stored: " << std::endl;
    std::map<std::string, double>::iterator itr;
    for ( itr = d_boundaries_map.begin(); itr != d_boundaries_map.end(); itr++ )
      std::cout << itr->first << ": " << itr->second << std::endl;

    // skip the data header line
    line_read.clear();
    std::getline(input_filestream, line_read);

    // read the actual data

    // initialize the array to store values (dynamic allocation, requires destructor)
    size_t lon_res = (size_t)d_boundaries_map.at("lon_res");
    size_t lat_res = (size_t)d_boundaries_map.at("lat_res");
    size_t depth_res = (size_t)d_boundaries_map.at("depth_res");
    d_location_values = new double ** [lon_res];
    for ( int idlo = 0; idlo < lon_res+1; idlo++ )
    {
      d_location_values[idlo] = new double*[lat_res];
      for ( int idla = 0; idla < lat_res+1; idla++ )
        d_location_values[idlo][idla] = new double[depth_res];
    }

    // do calculations to figure out indices
    double lon_step = std::abs(d_boundaries_map.at("lon_max") - d_boundaries_map.at("lon_min")) / lon_res;
    double lat_step = std::abs(d_boundaries_map.at("lat_max") - d_boundaries_map.at("lat_min")) / lat_res;
    double depth_step = std::abs(d_boundaries_map.at("depth_max") - d_boundaries_map.at("depth_min")) / depth_res;

    boost::tokenizer< boost::char_separator<char> >::iterator tok_itr;
    while ( std::getline(input_filestream, line_read) )
    {
      // nxt: split line, store values
      std::string lon, lat, depth, data;
      boost::tokenizer< boost::char_separator<char> > data_tok(line_read, sep);
      tok_itr = data_tok.begin();
      lon = *tok_itr;
      tok_itr++;
      lat = *tok_itr;
      tok_itr++;
      depth = *tok_itr;
      tok_itr++;
      data = *tok_itr;

      cnt++;

      size_t lon_idx, lat_idx, dep_idx;
      lon_idx = std::floor( ((double)atof(lon.c_str())-d_boundaries_map.at("lon_min"))/lon_step );
      lat_idx = std::floor( ((double)atof(lat.c_str())-d_boundaries_map.at("lat_min"))/lat_step );
      dep_idx = std::floor( ((double)atof(depth.c_str())-d_boundaries_map.at("depth_min"))/depth_step );

      d_location_values[lon_idx][lat_idx][dep_idx] = (double)atof(data.c_str());
    }

    // done with this file, close
    input_filestream.close();
  }
  else
    std::cout << "ERROR reading file: " << m_filename << std::endl;

  if ( cnt > 0 )
  {
    std::cout << "Done reading files, objects: " << cnt << '\n';
    m_file_read = true;
  }

  std::cout << "file read? " << m_file_read << std::endl;
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

  if (m_veh_lon >= lon_min && m_veh_lon <= lon_max &&
       m_veh_lat >= lat_min && m_veh_lat <= lat_max &&
       m_veh_depth >= dep_min && m_veh_depth <= dep_max )
  {
    // do calculations to figure out indices
    double lon_step = std::abs(lon_max - lon_min) / d_boundaries_map.at("lon_res");
    double lat_step = std::abs(lat_max - lat_min) / d_boundaries_map.at("lat_res");
    double depth_step = std::abs(dep_max - dep_min) / d_boundaries_map.at("depth_res");

    size_t nav_lon_idx, nav_lat_idx, nav_dep_idx;

    nav_lon_idx = std::floor( (m_veh_lon - lon_min) / lon_step );
    nav_lat_idx = std::floor( (m_veh_lat - lat_min) / lat_step );
    nav_dep_idx = std::floor( (m_veh_depth - dep_min) / depth_step );

    return d_location_values[nav_lon_idx][nav_lat_idx][nav_dep_idx];
  }
  // TODO: add boundary conditions (N,E,S,W)
  else
    return -1;
}
