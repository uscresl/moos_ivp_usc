/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SamplePoints.h                                       */
/*    DATE: Jan 25, 2016                                         */
/*                                                               */
/*****************************************************************/

#ifndef PSAMPLEPOINTS_HEADER
#define PSAMPLEPOINTS_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class SamplePoints : public CMOOSApp
{
 public:
   SamplePoints();
   ~SamplePoints() {};

 protected: 
   // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

   // Registration, Configuration, Mail handling utils
   void registerVariables();
   bool handleMailSamplePointsVarIn(std::string);

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
