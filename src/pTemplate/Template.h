/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: Template.h                                           */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*    Note: this is a template dir, so you can copy to start     */
/*          making your MOOSApp                                  */
/*                                                               */
/*****************************************************************/

#ifndef PTEMPLATE_HEADER
#define PTEMPLATE_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class Template : public CMOOSApp
{
 public:
   Template();
   ~Template() {};

 protected: 
   // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

   // Registration, Configuration, Mail handling utils
   void registerVariables();
   bool handleMailTemplateVarIn(std::string);

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
