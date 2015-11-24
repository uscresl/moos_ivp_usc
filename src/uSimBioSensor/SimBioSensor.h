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
   bool handleMailSimBioSensorVarIn(std::string);

 private: 
   // Own functions
   void runPython();
   void readBioDataFromFile();

   // Configuration variables
   std::string m_example1;
   double m_example2;
   // State variables
   double m_whatever;
   bool m_got_aabbcc;

   // temp hardcoded vars
   double m_min_lat;
   double m_max_lat;
   double m_min_lon;
   double m_max_lon;

   // data
   std::vector<DataPoint> m_data_pts;
};

#endif 
