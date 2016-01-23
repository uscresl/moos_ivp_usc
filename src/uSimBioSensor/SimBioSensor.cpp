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
//#include "ACTable.h"
#include "math.h"
#include <limits>

//// include python headers for running Python from C++
//// this generates some warning on compile, just ignore them
//// (the general advice way to get rid of the warnings is:
////  'include Python.h first, before other includes'
////  but even when put as first thing in header file, doesn't help here'
//#include <Python.h>

// read from file
#include <istream>
// atof
#include <stdlib.h>

// FLANN
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

// handle node report
#include "USCutils.h"

// csv handling
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

//using namespace std;

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
  // remove dynamically allocated array
  for ( int idlo = 0; idlo < d_lon_res+1; idlo++ )
  {
    for ( int idla = 0; idla < d_lat_res+1; idla++ )
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
  std::cout << "iterate, proceed? " << (m_file_read && m_nav_data_received) << std::endl;
  if ( m_file_read && m_nav_data_received )
  {
//    findClosestDataPoint();
    double dat = getDataPoint();
    std::cout << "data: " << dat << std::endl;
    if ( m_output_var != "" )
      m_Comms.Notify(m_output_var, dat);
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

//  // generate biomass file
//  runPython();

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
    // TODO read based on what header lines say,
    // for now, test quick knowing format
    // header lines
    // skip first header line, names
    std::getline(input_filestream, line_read);
    line_read.clear();

    // get the boundaries from header
    // (lon_min lon_max lon_res lat_min lat_max lat_res depth_min depth_max depth_res)
    std::getline(input_filestream, line_read);

    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> > tokens(line_read, sep);
    boost::tokenizer< boost::char_separator<char> >::iterator tok_itr = tokens.begin();
    d_lon_min = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_lon_max = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_lon_res = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_lat_min = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_lat_max = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_lat_res = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_depth_min = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_depth_max = (double)atof((*tok_itr).c_str());
    tok_itr++;
    d_depth_res = (double)atof((*tok_itr).c_str());

    std::cout << "boundaries stored: " << std::endl;
    std::cout << "lon: " << d_lon_min << ", " << d_lon_max << ", " << d_lon_res << std::endl;
    std::cout << "lat: " << d_lat_min << ", " << d_lat_max << ", " << d_lat_res << std::endl;
    std::cout << "dep: " << d_depth_min << ", " << d_depth_max << ", " << d_depth_res << std::endl;

    // skip the data header line
    line_read.clear();
    std::getline(input_filestream, line_read);

    // read the actual data

    // initialize the array to store values (dynamic allocation, requires destructor)
    d_location_values = new double ** [d_lon_res];
    for ( int idlo = 0; idlo < d_lon_res+1; idlo++ )
    {
      d_location_values[idlo] = new double*[d_lat_res];
      for ( int idla = 0; idla < d_lat_res+1; idla++ )
      {
        d_location_values[idlo][idla] = new double[d_depth_res];
      }
    }

    // do calculations to figure out indices
    double lon_step = std::abs(d_lon_max - d_lon_min) / d_lon_res;
    double lat_step = std::abs(d_lat_max - d_lat_min) / d_lat_res;
    double depth_step = std::abs(d_depth_max - d_depth_min) / d_depth_res;

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
      lon_idx = std::floor( ((double)atof(lon.c_str())-d_lon_min)/lon_step );
      lat_idx = std::floor( ((double)atof(lat.c_str())-d_lat_min)/lat_step );
      dep_idx = std::floor( ((double)atof(depth.c_str())-d_depth_min)/depth_step );

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
  if (m_veh_lon >= d_lon_min && m_veh_lon <= d_lon_max &&
       m_veh_lat >= d_lat_min && m_veh_lat <= d_lat_max &&
       m_veh_depth >= std::abs(d_depth_min) && m_veh_depth <= std::abs(d_depth_max) )
  {
    // do calculations to figure out indices
    double lon_step = std::abs(d_lon_max - d_lon_min) / d_lon_res;
    double lat_step = std::abs(d_lat_max - d_lat_min) / d_lat_res;
    double depth_step = std::abs(d_depth_max - d_depth_min) / d_depth_res;

    size_t nav_lon_idx, nav_lat_idx, nav_dep_idx;

    nav_lon_idx = std::floor( (m_veh_lon-d_lon_min)/lon_step );
    nav_lat_idx = std::floor( (m_veh_lat-d_lat_min)/lat_step );
    nav_dep_idx = std::floor( (m_veh_depth-d_depth_min)/depth_step );

    return d_location_values[nav_lon_idx][nav_lat_idx][nav_dep_idx];
  }
  // TODO: add boundary conditions (N,E,S,W)
  else
    return -1;
}

//void SimBioSensor::findClosestDataPoint() //Location vehicle, DataPoint & closest)
//{
//  // call FLANN
//  std::cout << "start" << std::endl;

//  size_t nn = m_locations.size();

//  std::cout << "reformat data" << std::endl;

//  // Matrix(T* data_, size_t rows_, size_t cols_, size_t stride_ = 0)
//  flann::Matrix<float> test(new float[nn*3], nn, 3);
//  for ( int idx = 0; idx < nn; ++idx )
//  {
//    Location loc = m_locations.at(idx);
//    // serialize data
//    test[idx][0] = loc.lon();
//    test[idx][1] = loc.lat();
//    test[idx][2] = loc.depth();
//  }

//  // AutotunedIndexParams(float target_precision = 0.8, float build_weight = 0.01, float memory_weight = 0, float sample_fraction = 0.1)
//  std::cout << "init index" << std::endl;
//  flann::Index<flann::L2<float> > index(test, flann::LinearIndexParams());
////                                        flann::KDTreeIndexParams(4));
////                                        flann::AutotunedIndexParams(0.8,0.01,0,0.1)); // 80% precision

//  std::cout << "build index" << std::endl;
//  index.buildIndex();

//  std::cout << "prep dists, indices" << std::endl;

//  // prep data structs for storing result, automatically resized as needed
//  std::vector< std::vector<int> > indices;
//  std::vector< std::vector<float> > dists;

//  flann::Matrix<float> query(new float[1*3], 1, 3);
//  query[0][0] = m_veh_lon;
//  query[0][1] = m_veh_lat;
//  query[0][2] = m_veh_depth;

//  std::cout << "run kNN" << std::endl;
//  // nr of nearest neighbors to search for
//  size_t k_neighbors = 1;
//  // run kNN, standard search
//  index.knnSearch(query, indices, dists, k_neighbors, flann::SearchParams(-1)); //flann::FLANN_CHECKS_AUTOTUNED) );
//  // returned are: indices of, and distances to, the neighbors found

//  size_t ind = (indices.at(0)).at(0);
//  Location pt_found(test[ind][0], test[ind][1], test[ind][2]);
//  double data = m_data_at_loc.at(pt_found);

//  std::cout << GetAppName() << "*best pt: " << test[ind][0] << ", " << test[ind][1] << ", "
//            << test[ind][2] << '\n';
//  std::cout << GetAppName() << "*value: " << data << std::endl;
//  // WORKING :D

//  if ( m_output_var != "" )
//    m_Comms.Notify(m_output_var, data);
//}
