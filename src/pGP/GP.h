/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: GP.h                                                 */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#ifndef PGP_HEADER
#define PGP_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

// lib GP
#include "gp.h"

class GP : public CMOOSApp
{
 public:
   GP();
   ~GP() {};

 protected: 
   // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

   // Registration, Configuration, Mail handling utils
   void registerVariables();
   bool handleMailGPVarIn(std::string);

 private: 
   // Own functions
   void handleMailData(double received_data);
   void storeSamplePoints(std::string input_string);

   // Configuration variables
   std::string m_input_var_data;
   std::string m_input_var_sample_points;
   std::string m_output_var_pred;
   size_t m_prediction_interval;

   // State variables
   double m_lat;
   double m_lon;
   double m_dep;
   bool m_data_added;

   std::vector< std::pair<double, double> > m_sample_points;

   libgp::GaussianProcess m_gp;
};

#endif 
