/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: NumVehicles.h                                        */
/*    DATE: Feb 25, 2015                                         */
/*                                                               */
/*    This process publishes the total number of vehicles for    */
/*    which node reports are received, and the names of the      */
/*    other vehicles.                                            */
/*                                                               */
/*****************************************************************/

#ifndef PTEMPLATE_HEADER
#define PTEMPLATE_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class NumVehicles : public CMOOSApp
{
 public:
   NumVehicles();
   ~NumVehicles() {};

 protected: 
   // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

   // Registration, Configuration, Mail handling utils
   void registerVariables();
   bool handleNodeReport(std::string const & node_report);

 private: 
   // Own functions
   void cleanVehicleMap();
   void publishNrVehicles();

   // Configuration variables
   double m_time_limit;

   // State variables
   std::map<std::string,double> m_other_vehicles;
   std::string m_own_name;
};

#endif 
