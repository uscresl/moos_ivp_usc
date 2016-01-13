/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: GP.cpp                                               */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#include "GP.h"

#include <iterator>
#include "MBUtils.h"
//#include "ACTable.h"
#include "math.h"
#include <limits>

using namespace std;

//---------------------------------------------------------
// Constructor
//
GP::GP()
{
  // class variable instantiations can go here
  m_example1 = "";
  m_example2 = -1;
  m_got_aabbcc = false;
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool GP::OnNewMail(MOOSMSG_LIST &NewMail)
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
    
    if( key == "GP_VAR_IN" ) 
    {
      handleMailGPVarIn(sval);
    }
    else if ( key == "GP_VAR_IN2" )
    {
      m_whatever = dval;
      // let's check if we can quit the application
      RequestQuit();
    }
    else
      std::cout << "pGP :: Unhandled Mail: " << key << std::endl;
      //reportRunWarning("Unhandled Mail: " + key);
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool GP::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GP::Iterate()
{
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GP::OnStartUp()
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
void GP::registerVariables()
{
  m_Comms.Register("GP_VAR_IN", 0);
  m_Comms.Register("GP_VAR_IN2", 0);
}

//---------------------------------------------------------
// Procedure: handleMailGPVarIn
//            a place to do more advanced handling of the
//            incoming message
//
bool GP::handleMailGPVarIn(string str)
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
    std::cout << GetAppName() << " :: Unhandled GPVarIn: " << original_msg << std::endl;
    //reportRunWarning("Unhandled GPVarIn:" + original_msg);

  if(valid_msg) 
  {
    m_got_aabbcc = true;
  }

  return(valid_msg);
}
