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

// std queue for FIFO queue
#include <queue>

class GP : public CMOOSApp
{
 public:
   GP();
   ~GP();

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
   void handleMailSamplePoints(std::string input_string);
   void handleMailSamplePointsSpecs(std::string input_string);
   void updateVisitedSet(double veh_lon, double veh_lat, size_t index );

   void addPatternToGP(double veh_lon, double veh_lat, double value);
   void dataAddingThread();

   bool runHPOptimization(libgp::GaussianProcess & gp, size_t nr_iterations);

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

   void publishNextBestPosition(); //Eigen::Vector2d best_so_far_y);

   void makeAndStorePredictions();

   void storeDataForSending(double vlon, double vlat, double data);
   void sendReady();
   void sendData();
   size_t handleMailReceivedDataPts(std::string incoming_data);

   // helper/test functions
   bool need_to_update_maps(size_t grid_index);
   int get_index_for_map(double veh_lon, double veh_lat);
   void checkDistanceToSampledPoint(double veh_lon, double veh_lat, Eigen::Vector2d move_pt);
   bool checkGPHasData();
   void calcLonLatSpacingAndBuffers();

   size_t processReceivedData();

   // Configuration variables
   std::string m_input_var_data;
   std::string m_input_var_sample_points;
   std::string m_input_var_sample_points_specs;
   std::string m_input_var_adaptive_trigger;
   std::string m_input_var_share_data;
   std::string m_output_var_pred;
   std::string m_output_filename_prefix;
   std::string m_output_var_share_data;

   size_t m_prediction_interval;
   bool m_use_MI;

   // State variables
   std::string m_veh_name;
   bool m_use_log_gp;
      // vehicle location
   double m_lat;
   double m_lon;
   double m_dep;
      // process state
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
   double m_pilot_done_time;
   double m_hp_optim_done_time;
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
   std::mutex m_file_writing_mutex;

   // create queue for adding of points to GP
   std::queue< std::vector<double> > m_queue_data_points_for_gp;

   // hyperparam optimization in multi-threading
   std::future<bool> m_future_hp_optim;
   bool m_hp_optim_running;
   bool m_hp_optim_done;

   // future for result MI criterion calculations
   std::future<Eigen::Vector2d> m_future_next_pt;

   // to add only every other data point
   size_t m_data_mail_counter;

   // mission states,
   // check when returning to base
   bool m_finished;
   size_t m_hp_optim_mode_cnt;

   // file writing
   std::ofstream m_ofstream_pm, m_ofstream_pv;
   std::ofstream m_ofstream_pmu, m_ofstream_psigma2;

   // nr of vehicles (for determining data exchange)
   size_t m_num_vehicles;

   // vectors for storing data for sending
   std::vector< std::string > m_data_to_send;
   size_t m_data_pt_counter;
   size_t m_data_send_reserve;
   bool m_received_shared_data;
   std::future<size_t> m_future_received_data_pts_added;
   std::vector<std::string> m_incoming_data_to_be_added;
   double m_loiter_dist_to_poly;

   // times data sharing
   bool m_timed_data_sharing;
   bool m_data_sharing_activated;
   bool m_sending_data;
   std::future<size_t> m_future_received_data_processed;
   bool m_waiting;
   bool m_received_ready;
   std::string m_input_var_handshake_data_sharing;
   std::string m_output_var_handshake_data_sharing;
   size_t m_last_ready_sent;
};

#endif 
