/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: GP_AUV.h                                             */
/*    DATE: 2015 - 2017                                          */
/*                                                               */
/*****************************************************************/

#ifndef PGP_AUV_HEADER
#define PGP_AUV_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

// lib GP
#include "gp.h"
// use unordered map rather than map, improve efficiency
#include <unordered_map>
// use unordered set for fast retrieval of keys in list
#include <unordered_set>

// multi-threading
#include <thread>
#include <future>

// std queue for FIFO queue
#include <queue>

// geodesy for conversion x/y to lon/lat
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

#include <Eigen/StdVector>

#include "GraphNode.h"

class GP_AUV : public CMOOSApp
{
 public:
   GP_AUV();
   ~GP_AUV();

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
   void initGeodesy();

   //// methods for handling incoming 'mail' //////////////////////////////////
   void handleMailData(double received_data);
   void handleMailSamplePoints(std::string input_string);
   void handleMailSamplePointsSpecs(std::string input_string);

   //// methods for handling sampled data /////////////////////////////////////
   void updateVisitedSet(double veh_lon, double veh_lat, size_t index );

   void addPatternToGP(double veh_lon, double veh_lat, double value);
   void dataAddingThread();

   //// methods for hyperparameter optimization ///////////////////////////////
   void startAndCheckHPOptim();
   bool runHPOptimization(size_t nr_iterations);
   void runHPoptimizationOnDownsampledGP(Eigen::VectorXd & loghp, size_t nr_iterations);

   //// methods for finding next waypoint /////////////////////////////////////
   void findAndPublishNextWpt();
   void kickOffCalcMetric();
   void getRandomStartLocation();
   // maximum entropy calculations
   size_t calcMECriterion();
   void getLogGPPredMeanVarFromGPMeanVar(double gp_mean, double gp_cov, double & lgp_mean, double & lgp_cov);
   // path planning & passing on to behavior
   void greedyWptSelection(std::string & next_waypoint);
   void publishNextWaypointLocations();

   // timed saving of GP  /////////////////////////////////////////////////////
   void makeAndStorePredictions();

   // helper/test functions  //////////////////////////////////////////////////
   bool needToUpdateMaps(size_t grid_index);
   int getIndexForMap(double veh_lon, double veh_lat);
   bool checkDistanceToSampledPoint(double veh_lon, double veh_lat, Eigen::Vector2d move_pt);
   bool checkGPHasData();
   void calcLonLatSpacing();
   bool convLonLatToUTM (double lon, double lat, double & lx, double & ly );
   bool convUTMToLonLat (double lx, double ly, double & lon, double & lat );
   bool inSampleRectangle(double veh_lon, double veh_lat, bool use_buffer) const;

   void tdsResetStateVars();

   size_t processReceivedData();

   bool ownMessage(std::string input);
   bool finalSurface(std::string input);

   void publishStates(std::string const calling_method);

   void endMission();

   void printCoutPrefix();

   // Configuration variables /////////////////////////////////////////////////
   bool m_verbose;

   CMOOSGeodesy m_geodesy;

   std::string m_input_var_data;
   std::string m_input_var_sample_points;
   std::string m_input_var_sample_points_specs;
   std::string m_input_var_adaptive_trigger;

   std::string m_output_var_pred;
   std::string m_output_filename_prefix;

   size_t m_prediction_interval;
   std::string m_path_planning_method;

   // State variables /////////////////////////////////////////////////////////
   bool m_debug;

   std::string m_veh_name;

   // use log gp or normal gp?
   bool m_use_log_gp;

   // vehicle location
   double m_lat;
   double m_lon;
   double m_dep;
   size_t m_surf_cnt;
   bool m_on_surface;

   // run lawnmower (passive, create GP, no wpt selection) or adaptive
   bool m_adaptive;

   // process state
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
   double m_y_resolution;
   double m_lon_deg_to_m;
   double m_lat_deg_to_m;

   // mission status
   double m_start_time;
   bool m_finding_nxt;

   // states through enums
   enum MissionState{ STATE_IDLE, STATE_CALCWPT, STATE_SAMPLE,
                      STATE_HPOPTIM, STATE_DONE,
                      STATE_SURFACING };
   MissionState m_mission_state;
   char const * currentMissionStateString() { return (char const *[]) {
   "STATE_IDLE", "STATE_CALCWPT", "STATE_SAMPLE", "STATE_HPOPTIM", "STATE_DONE",
   "STATE_SURFACING" }[m_mission_state]; };

   // maps for storing visited and unvisited sampling graph nodes (size_t index, GraphNode* graph_node)
   std::unordered_map< size_t, GraphNode* > m_sample_points_unvisited;
   std::unordered_map< size_t, GraphNode* > m_sample_points_visited;
   // vector for storing all sampling graph nodes
   std::vector< GraphNode > m_sample_graph_nodes;


   // GP, and create mutex for protection of parts of code accessing m_gp
   libgp::GaussianProcess * m_gp;
   std::mutex m_gp_mutex;
   std::mutex m_sample_maps_mutex;
   std::mutex m_file_writing_mutex;

   // create queue for adding of points to GP
   std::queue< std::vector<double> > m_queue_data_points_for_gp;
   std::queue< std::vector<double> > m_data_for_hp_optim;

   // hyperparam optimization in multi-threading
   std::future<bool> m_future_hp_optim;
   bool m_hp_optim_running;
   bool m_final_hp_optim;
   size_t m_hp_optim_iterations;
   bool m_hp_optim_cg;
   size_t m_last_hp_optim_done;

   // future for result MI criterion calculations
   std::future<size_t> m_future_next_pt;

   // to add only every other data point
   size_t m_data_mail_counter;

   // mission states,
   // check when returning to base
   bool m_finished;

   // file writing
   std::ofstream m_ofstream_pm_lGP, m_ofstream_pv_lGP;
   std::ofstream m_ofstream_pmu_GP, m_ofstream_psigma2_GP;

   double m_loiter_dist_to_poly;

   // downsampling data for HP optimization
   size_t m_downsample_factor;

   // do HP optim on first surface for adaptive
   bool m_first_surface;

   // buffer around area for which vehicle is counted to be inside the area
   double m_area_buffer;

   // keep track of bhv state
   std::string m_bhv_state;

   // debugging
   double m_db_uptime;
};

#endif
