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
   void makeYourOwn();

   // Configuration variables
   std::string m_example1;
   double m_example2;
   // State variables
   double m_whatever;
   bool m_got_aabbcc;
};

#endif 
