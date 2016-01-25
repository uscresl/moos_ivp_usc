/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: Template.cpp                                         */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*    Note: this is a template dir, so you can copy to start     */
/*          making your MOOSApp                                  */
/*          Nb. this is not an AppCastMOOSApp                    */
/*                                                               */
/*****************************************************************/

#include "Template.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

//---------------------------------------------------------
// Constructor
//
Template::Template()
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
bool Template::OnNewMail(MOOSMSG_LIST &NewMail)
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
      handleMailTemplateVarIn(sval);
    }
    else if ( key == "TEMPLATE_VAR_IN2" )
    {
      m_whatever = dval;
      // let's check if we can quit the application
      RequestQuit();
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Template::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Template::Iterate()
{
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Template::OnStartUp()
{
  CMOOSApp::OnStartUp();
  
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if ( !m_MissionReader.GetConfiguration(GetAppName(), sParams) )
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();

  STRING_LIST::iterator p;
  for ( p=sParams.begin(); p!=sParams.end(); p++ )
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if ( (param == "example2") && isNumber(value) )
    {
      // assuming the atof works, store the val
      m_example2 = atof(value.c_str());
      handled = true;

      std::cout << GetAppName() << " :: set m_example2 to be: " << m_example2 << std::endl;

      // let's check if we can quit the application, e.g. because we don't like
      // the param value
      if ( m_example2 < -99.9 )
      {
        std::cout << GetAppName() << " :: oh no, value is < -99.9" << std::endl;
        return(false); // this would be the preferred way to quit in OnStartUp
      }
      else if ( m_example2 > 99.9 )
      {
        std::cout << GetAppName() << " :: oh no, value is > 99.9" << std::endl;
        RequestQuit();
      }

    }
    else if( param == "example1" )
    {
      // save string .. you might wanna check for format or something
      m_example1 = value;
      handled = true;
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
void Template::registerVariables()
{
  m_Comms.Register("TEMPLATE_VAR_IN", 0);
  m_Comms.Register("TEMPLATE_VAR_IN2", 0);
}

//---------------------------------------------------------
// Procedure: handleMailTemplateVarIn
//            a place to do more advanced handling of the
//            incoming message
//
bool Template::handleMailTemplateVarIn(std::string str)
{
  // Expected parts in string:
  std::string aa, bb, cc;
  
  // Parse and handle ack message components
  bool valid_msg = true;
  std::string original_msg = str;
  // handle comma-separated string
  std::vector<std::string> svector = parseString(str, ',');
  unsigned int idx, vsize = svector.size();
  for ( idx = 0; idx < vsize; idx++ ) {
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

  if ( ( aa == "") || ( bb == "") || ( cc == "") )
    valid_msg = false;
  
  if ( !valid_msg )
    std::cout << GetAppName() << " :: Unhandled TemplateVarIn: " << original_msg << std::endl;

  if ( valid_msg )
    m_got_aabbcc = true;

  return ( valid_msg );
}
