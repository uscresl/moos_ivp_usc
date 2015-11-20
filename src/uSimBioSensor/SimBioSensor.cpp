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

using namespace std;

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
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

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

  runPython();

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
}

//---------------------------------------------------------
// Procedure: handleMailSimBioSensorVarIn
//            a place to do more advanced handling of the
//            incoming message
//
bool SimBioSensor::handleMailSimBioSensorVarIn(string str)
{
  // Expected parts in string:
  std::string aa, bb, cc;
  
  // Parse and handle ack message components
  bool   valid_msg = true;
  string original_msg = str;
  // handle comma-separated string
  vector<string> svector = parseString(str, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
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
  // very high level embedding
  // via (https://docs.python.org/2/extending/embedding.html)
  //Py_SetProgramName();
  Py_Initialize();
  //PyRun_SimpleFile("/home/stephanie/git/ipython-scripts-stephanie/testGMM.py");
  PyRun_SimpleString("from time import time, ctime\n"
                     "print 'Today is',ctime(time())\n");
  Py_Finalize();
}
