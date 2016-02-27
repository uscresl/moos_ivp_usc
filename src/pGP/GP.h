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
// use unordered map rather than map, improve efficiency
#include <unordered_map>

// multi-threading
#include <thread>
#include <future>

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
   void storeSamplePointsSpecs(std::string input_string);
   void updateVisitedSet(double * location);

   void addPatternToGP(double value, double * location);

   bool runHPOptimization(libgp::GaussianProcess & gp);

   void findNextSampleLocation();

   // mutual information
   Eigen::Vector2d calcMICriterion(libgp::CovarianceFunction& cov_f);
   void createCovarVector(libgp::CovarianceFunction& cov_f, Eigen::Vector2d y, std::string const & set_identifier, Eigen::VectorXd & k_ya);
   void createCovarMatrix(libgp::CovarianceFunction& cov_f, std::string const & set_identifier, Eigen::MatrixXd & K_aa);
   void getTgtValUnvisited(Eigen::VectorXd & t_av);
   //, size_t size_unvisited, Eigen::Vector2d & best_so_far_y, double & best_so_far);
   void logGPfromGP(double gp_mean, double gp_cov, double & lgp_mean, double & lgp_cov);

   // maximum entropy
   Eigen::Vector2d calcMECriterion();

   void publishNextBestPosition(Eigen::Vector2d best_so_far_y);

   void makeAndStorePredictions();

   // helper/test functions
   void checkDistanceToSampledPoint(double veh_lon, double veh_lat, Eigen::Vector2d move_pt);
   bool checkGPHasData();
   void calcLonLatSpacingAndBuffers();

   // Configuration variables
   std::string m_input_var_data;
   std::string m_input_var_sample_points;
   std::string m_input_var_sample_points_specs;
   std::string m_input_var_pilot_done;
   std::string m_input_var_adaptive_trigger;
   std::string m_output_var_pred;
   std::string m_output_filename_prefix;

   size_t m_prediction_interval;
   bool m_use_MI;

   // State variables
      // vehicle location
   double m_lat;
   double m_lon;
   double m_dep;
      // process state
//   bool m_data_added;
   bool m_pause_data_adding;
   double m_last_published;
   double m_last_pred_save;
      // sample points grid specs
   double m_min_lon;
   double m_min_lat;
   double m_max_lon;
   double m_max_lat;
   double m_pts_grid_width;
   double m_pts_grid_height;
   double m_pts_grid_spacing;
   double m_lon_spacing;
   double m_lat_spacing;
   double m_buffer_lon;
   double m_buffer_lat;
   double m_y_resolution;
   double m_lon_deg_to_m;
   double m_lat_deg_to_m;
   // mission status
   bool m_pilot_done;
   double m_pilot_done_time;
   bool m_need_nxt_wpt;
   bool m_finding_nxt;

   //std::vector< std::pair<double, double> > m_sample_points;
   std::unordered_map< size_t, Eigen::Vector2d > m_sample_points_unvisited;
   std::unordered_map< size_t, Eigen::Vector2d > m_sample_points_visited;
   std::vector< std::pair<double, double> > m_sample_locations;

   // GP, and create mutex for protection of parts of code accessing m_gp
   libgp::GaussianProcess m_gp;
   std::mutex m_gp_mutex;
   std::mutex m_sample_maps_mutex;

   // hyperparam optimization in multi-threading
   std::future<bool> m_future_hp_optim;
   bool m_hp_optim_running;
   bool m_hp_optim_done;

   // future for result MI criterion calculations
   std::future<Eigen::Vector2d> m_future_next_pt;
};

#endif 
