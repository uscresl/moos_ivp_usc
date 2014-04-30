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
#include "math.h"
#include <limits>

#include <Eigen/Dense>
#include "hungarianmethod.h"

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
  m_ownship = "";
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
    else if ( key == "NODE_REPORT" )
    { // need to get all vehicle data for HM
      // std::map<std::string,std::string> m_other_vehicles;
      std::string veh_name = getStringFromNodeReport(sval, "NAME");
      std::map<std::string,std::string>::iterator found_iterator = m_other_vehicles.find(veh_name);
      if ( found_iterator != m_other_vehicles.end() )
      { // update value
        m_other_vehicles[veh_name] = sval;
      }
      else
      { // maybe insert vehicle
        // check not own name
        if ( veh_name != m_ownship && veh_name != m_lead_vehicle)
          m_other_vehicles.insert(std::pair<std::string, std::string>(veh_name, sval));
      }
    }
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

  if(!m_MissionReader.GetValue("Community", m_ownship))
  {
    std::cout << "Vehicle Name (MOOS community) not provided" << std::endl;
    return(false);
  }

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) 
  {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if ( param == "lead_vehicle_name" )
    {
      m_lead_vehicle= value;
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
void PositionInFormation::registerVariables()
{
  m_Comms.Register("DESIRED_FORMATION", 0);
  m_Comms.Register("FORMATION_SHAPE", 0);
  m_Comms.Register("NAV_X",0);
  m_Comms.Register("NAV_Y",0);
  m_Comms.Register("NAV_Z",0);
  m_Comms.Register("NODE_REPORT",0); // need to get all vehicle data for HM
}

void PositionInFormation::findPosition()
{
  // examples: "2700,1880" or 
  //           "2689.5,1880:2710.5,1880" or 
  //           "2700,1880:2679,1859:2721,1859"

  // gotta get all positions from the formation
  vector<string> svector = parseString(m_formation, ':');
  unsigned int idx, vsize = svector.size();

  // only need to calculate if there are more than 1 vehicle following
  if ( m_other_vehicles.size() >= 1 && vsize >= 1 )
  {
    // constract Eigen matrix (num_vehicles*num_positions)
    Eigen::MatrixXd hm_matrix( m_other_vehicles.size()+1, vsize);

    // need distances for Hungarian method cost matrix
    double xval, yval; // TODO make this 3D
    for (idx = 0; idx < vsize; idx++)
    { // own vehicle calculations
      std::cout << "dealing with: " << svector[idx] << std::endl;
      size_t comma_at = svector[idx].find(',');
      std::string xstr = svector[idx].substr(0,comma_at-1);
      std::string ystr = svector[idx].substr(comma_at+1,svector[idx].length());
      xval = atof(xstr.c_str());
      yval = atof(ystr.c_str());
      std::cout << "xval, yval of pt " << idx+1 << ": " << xval << "," << yval << std::endl;
      std::cout << "vehicle at: " << m_x << "," << m_y << std::endl;
      
      // check&store distance to position
      double euclidD;    // TODO make this 3D
      euclidDistance(xval, yval, m_x, m_y, euclidD);
      // store distance metric into matrix
      hm_matrix(0,idx) = euclidD;
      std::cout << "calculated distance: " << hm_matrix(0,idx) << std::endl;
    }
    // for all other vehicles, calculate distance to all positions in formation
    std::map<std::string,std::string>::iterator vehicle_iter;
    size_t vnum = 1;
    for ( vehicle_iter = m_other_vehicles.begin(); vehicle_iter != m_other_vehicles.end(); ++vehicle_iter )
    { // for each vehicle
      std::string sval = vehicle_iter->second;
      double vx = getDoubleFromNodeReport(sval, "X");
      double vy = getDoubleFromNodeReport(sval, "Y");
      std::cout << "vehicle at: " << vx << "," << vy << std::endl;

      for (idx = 0; idx < vsize; idx++)
      { // for each point in formation
        size_t comma_at = svector[idx].find(',');
        std::string xstr = svector[idx].substr(0,comma_at-1);
        std::string ystr = svector[idx].substr(comma_at+1,svector[idx].length());
        xval = atof(xstr.c_str());
        yval = atof(ystr.c_str());
        std::cout << "xval, yval of pt " << idx+1 << ": " << xval << "," << yval << std::endl;

        double euclidD;
        euclidDistance(xval, yval, vx, vy, euclidD);
        // store distance metric into matrix
        hm_matrix(vnum, idx) = euclidD;
        std::cout << "calculated distance for vehicle " << vnum << ": " << hm_matrix(vnum,idx) << std::endl;
      }
      vnum++;
    }
    std::cout << hm_matrix << std::endl;

    // pass on matrix to hungarian method solve function for optimal assignment
    HungarianMethod hu_method;
    Eigen::VectorXi hu_optimal_assignment;
    hu_optimal_assignment = hu_method.hungarian_solve(hm_matrix);
    std::cout << "optimal assignment: " << hu_optimal_assignment << std::endl;
    
    // extract assignment for current vehicle & publish
    size_t hm_optimal_position = hu_optimal_assignment[0]+1;
    Notify("POSITION_IN_FORMATION",hm_optimal_position);
  }
}
