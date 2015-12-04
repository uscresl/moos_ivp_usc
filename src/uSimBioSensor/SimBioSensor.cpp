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

// include python headers for running Python from C++
// this generates some warning on compile, just ignore them
// (the general advice way to get rid of the warnings is:
//  'include Python.h first, before other includes'
//  but even when put as first thing in header file, doesn't help here'
#include <Python.h>

// read from file
#include <istream>
// atof
#include <stdlib.h>

// FLANN
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

// handle node report
#include "USCutils.h"

//using namespace std;

//---------------------------------------------------------
// Constructor
//
SimBioSensor::SimBioSensor()
{
  // class variable instantiations can go here
  m_example1 = "";
  m_example2 = -1;
  m_got_aabbcc = false;

  // temporary hardcoded values for initial testing
  // TODO take from input WKT polygon or similar
  m_min_lat = 34.0784000000111;
  m_max_lat = 34.0935;
  m_min_lon = -117.815;
  m_max_lon = -117.793099999973;

  m_veh_lon = 0;
  m_veh_lat = 0;
  m_veh_depth = 0;

//  m_data_pts = new std::vector<DataPoint>();

  m_test = 0;
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
    
    if( key == "SIMBIOSENSOR_VAR_IN" ) 
    {
      handleMailSimBioSensorVarIn(sval);
    }
    else if ( key == "SIMBIOSENSOR_VAR_IN2" )
    {
      m_whatever = dval;
      // let's check if we can quit the application
      RequestQuit();
    }
    else if ( key == "NODE_REPORT_LOCAL" )
    {
      //handle
      m_veh_lon = getDoubleFromNodeReport(sval,"LON");
      m_veh_lat = getDoubleFromNodeReport(sval,"LAT");
      m_veh_depth = getDoubleFromNodeReport(sval,"DEP");

      if ( m_test == 0 )
        m_test = 1;
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
  std::cout << "iterate, test = " << m_test << std::endl;
  if ( m_test == 1 )
  {
    findClosestDataPoint();
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
    std::cout << GetAppName() << " :: No config which is not allowed because non-const member functions make NO PROMISE not to modify the obblock found for " << GetAppName();
    //reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) 
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if((param == "example2") && isNumber(value)) 
    {
      // assuming the atof works, store the val
      m_example2 = atof(value.c_str());
      handled = true;

      std::cout << GetAppName() << " :: set m_example2 to be: " << m_example2 << std::endl;

      // let's check if we can quit the application, e.g. because we don't like
      // the param value
      if (m_example2 < -99.9)
      {
        std::cout << GetAppName() << " :: oh no, value is < 99.9" << std::endl;
        return(false); // this would be the preferred way to quit in OnStartUp
      }
      else if (m_example2 > 99.9)
      {
        std::cout << GetAppName() << " :: oh no, value is > 99.9" << std::endl;
        RequestQuit();
      }

    }
    else if( (param == "example1") ) 
    {
      // save string .. you might wanna check for format or something
      m_example1 = value;
      handled = true;
    }
    //TODO add parameter for reading in WKT polygon, rather than hardcoded
    // min/max lat/lon

    if(!handled)
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
      //reportUnhandledConfigWarning(orig);
  }

  // generate biomass file
  runPython();

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
  m_Comms.Register("SIMBIOSENSOR_VAR_IN", 0);
  m_Comms.Register("SIMBIOSENSOR_VAR_IN2", 0);

  // get current vehicle position
  // because we want lon/lat/depth at the same time, we might as well use
  // the node report
  m_Comms.Register("NODE_REPORT_LOCAL", 0);
}

//---------------------------------------------------------
// Procedure: handleMailSimBioSensorVarIn
//            a place to do more advanced handling of the
//            incoming message
//
bool SimBioSensor::handleMailSimBioSensorVarIn(std::string str)
{
  // Expected parts in string:
  std::string aa, bb, cc;
  
  // Parse and handle ack message components
  bool   valid_msg = true;
  std::string original_msg = str;
  // handle comma-separated string
  std::vector<std::string> svector = parseString(str, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    std::string param = biteStringX(svector[i], '=');
    std::string value = svector[i];
    if(param == "aa")
      aa = value;
    else if(param == "bb")
      bb = value;
    else if(param == "cc")
      cc = value;
    else
      valid_msg = false;       
  }

  if( (aa=="") || (bb=="") || (cc=="") )
    valid_msg = false;
  
  if(!valid_msg)
    std::cout << GetAppName() << " :: Unhandled SimBioSensorVarIn: " << original_msg << std::endl;
  std::cout << GetAppName() << " :: Unhandled SimBioSensorVarIn: " << original_msg << std::endl;
  std::cout << GetAppName() << " :: Unhandled SimBioSensorVarIn: " << original_msg << std::endl;
    std::cout << GetAppName() << " :: Unhandled SimBioSensorVarIn: " << original_msg << std::endl;
    //reportRunWarning("Unhandled SimBioSensorVarIn:" + original_msg);

  if(valid_msg) 
  {
    m_got_aabbcc = true;
  }

  return(valid_msg);
}

// own functions ///////////////////////////////////////////////////////////////

void SimBioSensor::runPython()
{
  // python embedding,
  // via (https://docs.python.org/2/extending/embedding.html)

  Py_Initialize();

  // filename
  // TODO make filename a parameter
  // note, make sure the path is part of PYTHONPATH, or otherwise set here
  PyObject *pFileName = PyUnicode_FromString((char *) "testGMM");
  PyObject *pModule = PyImport_Import(pFileName);
  // free memory name, now that we have module
  Py_DECREF(pFileName);

  PyObject *pFunc, *pArgs;
  // continue to call function in file
  if ( pModule != NULL )
  {
    // choose function

    // TODO make function name parameter
    pFunc = PyObject_GetAttrString(pModule, (char *)"create_gmm_and_save_to_file");

    if ( pFunc && PyCallable_Check(pFunc) )
    {
      // TODO: input arguments
      //      PyObject  *args = PyTuple_New(5);

      // no function arguments, so we can skip that for now
      // no return argument, so skip that as well
      PyObject_CallObject(pFunc, NULL);
    }
    // free up memory
    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
  }
  else
  {
    // print error, exit
    PyErr_Print();
    std::exit(0);
  }

  Py_Finalize();
}

void SimBioSensor::readBioDataFromFile()
{
  // read from file
  // data formatting: tab separated lon, lat, depth, bio_value
  // latitude: north - south
  // longitude: east - west
  std::ifstream input_filestream;
  // TODO make filename parameter
  input_filestream.open("test.out", std::ios::in);

  std::string line_read;
  std::istringstream line_stream;
  if ( input_filestream.is_open() )
  {
    while ( std::getline(input_filestream, line_read) )
    {
      // nxt: split line, store values
      std::string lon, lat, depth, data;
      line_stream.clear();
      line_stream.str(line_read);
      if ( line_stream >> lon >> lat >> depth >> data )
      {
        // store the data
        Location tmp( atof(lon.c_str()), atof(lat.c_str()), atof(depth.c_str()) );
        m_locations.push_back( tmp );
        m_data_at_loc.insert( std::pair<Location,double>(tmp, (double)atof(data.c_str())) );
      }
    }

    // done with this file, close
    input_filestream.close();
  }
  else
    std::cout << "error reading file" << std::endl;

  std::cout << "done reading files, objects: " << m_locations.size() << '\n';

  if ( m_veh_lat != 0 )
    m_test = 1;
}

void SimBioSensor::findClosestDataPoint() //Location vehicle, DataPoint & closest)
{
  // call FLANN
  std::cout << "start" << std::endl;

  size_t nn = m_locations.size();

  std::cout << "reformat data" << std::endl;

  // Matrix(T* data_, size_t rows_, size_t cols_, size_t stride_ = 0)
  flann::Matrix<float> test(new float[nn*3], nn, 3);
  for ( int idx = 0; idx < nn; ++idx )
  {
    Location loc = m_locations.at(idx);
    // serialize data
    test[idx][0] = loc.lon();
    test[idx][1] = loc.lat();
    test[idx][2] = loc.depth();
  }

/*
  C code:
  //flann_build_index_double(dataset, n rows, 3 columns, speedup?, parameters)
  float * speedup;
  flann_build_index_double(test, nn, 3, speedup, parameters)
*/

  // AutotunedIndexParams(float target_precision = 0.8, float build_weight = 0.01, float memory_weight = 0, float sample_fraction = 0.1)
  std::cout << "init index" << std::endl;
  flann::Index<flann::L2<float> > index(test, flann::LinearIndexParams());
//                                        flann::KDTreeIndexParams(4));
//                                        flann::AutotunedIndexParams(0.8,0.01,0,0.1)); // 80% precision

  std::cout << "build index" << std::endl;
  index.buildIndex();

  std::cout << "prep dists, indices" << std::endl;

  // prep data structs for storing result, automatically resized as needed
  std::vector< std::vector<int> > indices;
  std::vector< std::vector<float> > dists;

  std::cout << "create query point" << std::endl;
  // some random test point TODO take AUV position
  flann::Matrix<float> query(new float[1*3], 1, 3);
  query[0][0] = m_veh_lon;
  query[0][1] = m_veh_lat;
  query[0][2] = m_veh_depth;
  std::cout << "query created with " << query.rows << " entries" << std::endl;
  std::cout << "content: " << m_veh_lon << " " << m_veh_lat << " " << m_veh_depth << '\n' << std::endl;

  std::cout << "run kNN" << std::endl;
  // nr of nearest neighbors to search for
  size_t k_neighbors = 1;
  // run kNN, standard search
  index.knnSearch(query, indices, dists, k_neighbors, flann::SearchParams(-1)); //flann::FLANN_CHECKS_AUTOTUNED) );
  // returned are: indices of, and distances to, the neighbors found

  std::cout << "*dist: " << (dists.at(0)).at(0) << '\n';
  size_t ind = (indices.at(0)).at(0);
  std::cout << "*indi: " << ind << '\n';

  std::cout << "*best pt: " << test[ind][0] << ", " << test[ind][1] << ", "
            << test[ind][2] << std::endl;

  Location pt_found(test[ind][0], test[ind][1], test[ind][2]);
  double data = m_data_at_loc.at(pt_found);

  std::cout << "value: " << data << std::endl;
  // WORKING :D

  m_Comms.Notify("SIM_DATA", data);

  m_test = 0;
}
