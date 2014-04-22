/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: PositionInFormation.cpp                              */
/*    DATE: Apr 21, 2014                                         */
/*                                                               */
/*                                                               */
/*    This process will choose what position in the formation    */
/*    the vehicles should take, based on which one is closest.   */
/*                                                               */
/*    Nb. this is not an AppCastMOOSApp                          */
/*                                                               */
/*****************************************************************/

#include "PositionInFormation.h"

#include <iterator>
#include "MBUtils.h"
#include "math.h"
#include <limits>

using namespace std;

//---------------------------------------------------------
// Constructor
//
PositionInFormation::PositionInFormation()
{
  // class variable instantiations can go here
  m_x = 0;
  m_y = 0;
  m_z = 0;
  m_formation = "";
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool PositionInFormation::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if ( key == "DESIRED_FORMATION" ) 
      m_formation = sval;
    else if ( key == "FORMATION_SHAPE" )
    { // shape changed, re-determine position in shape
      findPosition();
    }
    else if ( key == "NAV_X" )
      m_x = dval;
    else if ( key == "NAV_Y" )
      m_y = dval;
    else if ( key == "NAV_Z" )
      m_z = dval;
    else
      std::cout << "pPositionInFormation :: Unhandled Mail: " << key << std::endl;
      //reportRunWarning("Unhandled Mail: " + key);
  }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool PositionInFormation::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PositionInFormation::Iterate()
{
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PositionInFormation::OnStartUp()
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
//    if((param == "example2") && isNumber(value)) 
//    {
//      // assuming the atof works, store the val
//      m_example2 = atof(value.c_str());
//      handled = true;
//    }
//    else if( (param == "example1") ) 
//    {
//      // save string .. you might wanna check for format or something
//      m_example1 = value;
//      handled = true;
//    }

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
void PositionInFormation::registerVariables()
{
  m_Comms.Register("DESIRED_FORMATION", 0);
  m_Comms.Register("FORMATION_SHAPE", 0);
  m_Comms.Register("NAV_X",0);
  m_Comms.Register("NAV_Y",0);
  m_Comms.Register("NAV_Z",0);
}

void PositionInFormation::findPosition()
{
  // examples: "2700,1880" or 
  //           "2689.5,1880:2710.5,1880" or 
  //           "2700,1880:2679,1859:2721,1859"

  // gotta get all positions from formation
  vector<string> svector = parseString(m_formation, ':');
  unsigned int idx, vsize = svector.size();
  double distances[vsize];
  double xval, yval;
  for (idx = 0; idx < vsize; idx++)
  {
    std::string xstr = biteStringX(svector[idx], ',');
    std::string ystr = svector[idx];
    xval = atof(xstr.c_str());
    yval = atof(ystr.c_str());
    // TODO make this 3D
    
    // check&store distance to position // TODO make this Hungarian method?
    double dx, dy;
    dx = xval - m_x;
    dy = yval - m_y;
    // TODO make this 3D
    distances[idx] = sqrt(dx*dx + dy*dy);
    std::cout << "calculated distance: " << distances[idx] << std::endl;
  }
  
  // then publish identifier
  double min_dist = std::numeric_limits<double>::max();
  size_t min_dist_idx = 0;
  for ( idx = 0; idx < vsize; idx++ )
  {
    if ( distances[idx] < min_dist )
    {
      min_dist = distances[idx];
      min_dist_idx = idx+1;
    }
  }
  Notify("POSITION_IN_FORMATION",min_dist_idx);
}
