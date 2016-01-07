/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SimBioSensor.h                                       */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#ifndef PSIMBIOSENSOR_HEADER
#define PSIMBIOSENSOR_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "DataPoint.h"

class SimBioSensor : public CMOOSApp
{
 public:
   SimBioSensor();
   ~SimBioSensor() {};

 protected: 
   // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

   // Registration, Configuration, Mail handling utils
   void registerVariables();

 private: 
   // Own functions
   void runPython();
   void readBioDataFromFile();
   void findClosestDataPoint(); //Location vehicle, DataPoint& closest);

   // Configuration variables
   std::string m_filename;

   // State variables
   bool m_file_read;
   bool m_nav_data_received;

   // MOOS vars
   double m_veh_lon;
   double m_veh_lat;
   double m_veh_depth;

   // data read from file
   std::vector<Location> m_locations;
   std::map<Location, double> m_data_at_loc;

};

#endif 
