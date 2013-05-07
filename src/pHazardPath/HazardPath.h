/*****************************************************************/
/*    NAME: Stephanie Kemna and Supreeth Subbaraya               */
/*    ORGN: Dept of Computer Science, USC                        */
/*    FILE: HazardPath.h                                         */
/*    DATE: Apr 19th 2013                                        */
/*                                                               */
/*****************************************************************/

#ifndef UFLD_HAZARD_PATH_HEADER
#define UFLD_HAZARD_PATH_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class HazardPath : public AppCastingMOOSApp
{
 public:
   HazardPath();
   ~HazardPath() {};

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Registration, Configuration, Mail handling utils
   void registerVariables();
   bool handleMailSensorConfigAck(std::string);

 private: // Own functions
   void postWaypointUpdate();
   void calculateSurveyArea();
   void postWaypointFollow();
   void handleHazardReport(std::string sval);

 private: // Configuration variables
   double m_number_of_vehicles;
   double m_coordinate_1x , m_coordinate_1y;
   double m_coordinate_2x , m_coordinate_2y;
   double m_coordinate_3x , m_coordinate_3y;
   double m_coordinate_4x , m_coordinate_4y;
   double m_lane_width_overlap;
   double m_survey_area_x , m_survey_area_y;
   double m_survey_area_width;
   double m_survey_area_height;
   double m_survey_lane_width;
   //bool m_survey_area_location;
   size_t m_survey_area_location;
   bool m_survey_order;

 private: // State variables
   // sensor params
   double m_swath_width_granted;
   double m_pd_granted;
   double m_pfa;
   double m_pclass;

   // survey - general
   std::string m_survey_mode;
   size_t m_surveys_done;
   
   // if the mode is lawnmower, nr of surveys to do
   size_t m_num_surveys;
   double m_start_lm_x;
   double m_start_lm_y;
   
   // if the mode is follow, calculate waypoints
   std::list< std::pair<double,double> > m_survey_waypoints;
   std::list< std::pair<double,double> > m_survey_waypoints_second;
   std::pair<double,double> m_previous_wpt;
   bool m_first_wpt;
   
   // to calculate when to return
   double m_mission_time;
   double m_return_threshold;
   // store the start location
   double m_start_x;
   double m_start_y;
   // current location
   double m_x;
   double m_y;
   double m_speed;
};

#endif 
