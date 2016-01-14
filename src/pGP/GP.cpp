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

#include "math.h"
#include <limits>

// lib GP
#include "gp.h"


//---------------------------------------------------------
// Constructor
//
GP::GP()
{
  // class variable instantiations can go here
  m_got_aabbcc = false;
  m_input_var = "";
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
    
    if( key == m_input_var )
    {
      //handleMailGPVarIn(sval);
      std::cout << "receiving: " << dval << " " << atof( sval.c_str() ) << std::endl;
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
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();
  //reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p=sParams.begin(); p!=sParams.end(); p++)
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if ( param == "input_var" )
    {
      m_input_var = toupper(value);
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
  if ( m_input_var != "" )
    m_Comms.Register(m_input_var, 0);
}

//---------------------------------------------------------
// Procedure: handleMailGPVarIn
//            a place to do more advanced handling of the
//            incoming message
//
bool GP::handleMailGPVarIn(std::string str)
{
  // Expected parts in string:
  std::string aa, bb, cc;

  // Parse and handle ack message components
  bool   valid_msg = true;
  std::string original_msg = str;
  // handle comma-separated string
  std::vector<std::string> svector = parseString(str, ',');
  unsigned int idx, vsize = svector.size();
  for ( idx=0; idx<vsize; idx++ ) {
    std::string param = biteStringX(svector[idx], '=');
    std::string value = svector[idx];
    if ( param == "aa" )
      aa = value;
    else if ( param == "bb" )
      bb = value;
    else if ( param == "cc" )
      cc = value;
    else
      valid_msg = false;
  }

  if ( (aa=="") || (bb=="") || (cc=="") )
    valid_msg = false;

  if ( !valid_msg )
    std::cout << GetAppName() << " :: Unhandled TemplateVarIn: " << original_msg << std::endl;
  //reportRunWarning("Unhandled TemplateVarIn:" + original_msg);

  if ( valid_msg )
    m_got_aabbcc = true;

  return ( valid_msg );
}
