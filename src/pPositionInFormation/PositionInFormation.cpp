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

  debug = true;
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
    {
      m_formation = sval;
      findPosition();
    }
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
      std::string veh_name = getStringFromNodeReport(sval, "NAME");
      std::map<std::string,std::string>::iterator found_iterator = m_other_vehicles.find(veh_name);
      if ( found_iterator != m_other_vehicles.end() )
      { // vehicle already in map, update value
        m_other_vehicles[veh_name] = sval;
        findPosition();
      }
      else
      { // insert vehicle, if not self or lead (should not receive node_report from self)
        if ( veh_name != m_ownship && veh_name != m_lead_vehicle)
          m_other_vehicles.insert(std::pair<std::string, std::string>(veh_name, sval));
        findPosition();
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
//  // if we solve the Hungarian Method only once (at time of an update),
//  // then, if the vehicles are close to each other, and given delays in
//  // position updates (acomms), they may solve different problems and
//  // still end up deciding to go to the same point.
//  size_t remainder = (size_t)(round(MOOSTime())) % 2;
//  if ( remainder == 0 )
//    findPosition();

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
  vector<string> positionsInFormationvector = parseString(m_formation, ':');
  unsigned int idx, nrPositions = positionsInFormationvector.size();

  // only need to calculate if there are more than 1 vehicle following
  if ( m_other_vehicles.size() >= 1 && nrPositions >= 1 )
  {
    // construct Eigen matrix (total_num_vehicles*num_positions)
    Eigen::MatrixXd hm_matrix( m_other_vehicles.size()+1, nrPositions);

    // need distances for Hungarian method cost matrix
    // TODO make this 3D
    for (idx = 0; idx < nrPositions; idx++)
    { // own vehicle calculations
      // for each possible position, calculate Euclidean distance to it
      double euclidD;
      euclidDistanceFromString(positionsInFormationvector[idx], m_x, m_y, euclidD);
      // store distance metric into matrix
      hm_matrix(0,idx) = euclidD;

      if (debug)
      {
        std::cout << "OWNSHIP CALC\n";
        std::cout << "vehicle at: " << m_x << "," << m_y << std::endl;
        std::cout << "calculating for position: " << positionsInFormationvector[idx] << std::endl;
        std::cout << "calculated euclid distance: " << hm_matrix(0,idx) << std::endl;
      }
    }
    // for all other vehicles, calculate distance to all positions in formation
    std::map<std::string,std::string>::iterator vehicle_iter;
    size_t vnum = 1;
    for ( vehicle_iter = m_other_vehicles.begin(); vehicle_iter != m_other_vehicles.end(); ++vehicle_iter )
    { // for each vehicle

      // get vehicle position
      std::string sval = vehicle_iter->second;
      double vx = getDoubleFromNodeReport(sval, "X");
      double vy = getDoubleFromNodeReport(sval, "Y");

      for (idx = 0; idx < nrPositions; idx++)
      { // for each possible position, calculate Euclidean distance to it
        double euclidD;
        euclidDistanceFromString(positionsInFormationvector[idx], vx, vy, euclidD);
        // store distance metric into matrix
        hm_matrix(vnum, idx) = euclidD;

        if (debug)
        {
          std::cout << "OTHER CALC\n";
          std::cout << "(other) vehicle at: " << vx << "," << vy << std::endl;
          std::cout << "calculating for position: " << positionsInFormationvector[idx] << std::endl;
          std::cout << "calculated euclid distance for vehicle " << vnum+1 << ": " << hm_matrix(vnum,idx) << std::endl;
        }
      }
      vnum++;
    }
    if (debug)
      std::cout << hm_matrix << std::endl;

    // pass on matrix to hungarian method solve function for optimal assignment
    HungarianMethod hu_method;
    Eigen::VectorXi hu_optimal_assignment;
    hu_optimal_assignment = hu_method.hungarian_solve(hm_matrix);
    if (debug)
      std::cout << "optimal assignment: " << hu_optimal_assignment << std::endl;
    
    // extract assignment for current vehicle & publish
    size_t hm_optimal_position = hu_optimal_assignment[0]+1;
    Notify("POSITION_IN_FORMATION",hm_optimal_position);
  }
}

void PositionInFormation::euclidDistanceFromString(std::string const & xy_str, double vehicle_x, double vehicle_y, double & euclidD)
{
  // get the individual values
  size_t comma_at = xy_str.find(',');
  std::string xstr = xy_str.substr(0,comma_at-1);
  std::string ystr = xy_str.substr(comma_at+1,xy_str.length());
  double xval, yval;
  xval = atof(xstr.c_str());
  yval = atof(ystr.c_str());

  // calculate the Euclid distance, return by argument
  euclidDistance(xval, yval, vehicle_x, vehicle_y, euclidD);
}
