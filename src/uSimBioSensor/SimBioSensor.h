/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: SimBioSensor.h                                       */
/*    DATE: Jan 25, 2016                                         */
/*                                                               */
/*****************************************************************/

#ifndef PSIMBIOSENSOR_HEADER
#define PSIMBIOSENSOR_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class SimBioSensor : public CMOOSApp
{
 public:
   SimBioSensor();
   ~SimBioSensor();

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
   void readBioDataFromFile();
   double getDataPoint();

   // MOOS vars
   double m_veh_lon;
   double m_veh_lat;
   double m_veh_depth;

   bool m_new_lon;
   bool m_new_lat;
   bool m_new_dep;

   // Configuration variables
   std::string m_filename;
   std::string m_output_var;

   // State variables
   bool m_file_read;
   bool m_nav_data_received;

   double m_lon_step;
   double m_lat_step;
   double m_depth_step;

   // data read from file
   std::map<std::string, double> d_boundaries_map; // lon_min lon_max lon_res
                                                   // lat_min lat_max lat_res
                                                   // depth_min depth_max depth_res
   double *** d_location_values; // lon, lat, dep -> data value

};

#endif 
