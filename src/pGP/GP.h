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
   void updateVisitedSet();

   void findNextSampleLocation();
   void createCovarVector(libgp::CovarianceFunction& cov_f, Eigen::VectorXd y, std::string const & set_identifier, Eigen::VectorXd & k_ya);
   void createCovarMatrix(libgp::CovarianceFunction& cov_f, std::string const & set_identifier, Eigen::MatrixXd & K_aa);

   bool runHPOptimization(libgp::GaussianProcess & gp);

   // Configuration variables
   std::string m_input_var_data;
   std::string m_input_var_sample_points;
   std::string m_input_var_sample_points_specs;
   std::string m_output_var_pred;
   size_t m_prediction_interval;

   // State variables
      // vehicle location
   double m_lat;
   double m_lon;
   double m_dep;
      // process state
   bool m_data_added;
   double m_last_published;
      // sample points grid specs
   double m_min_lon;
   double m_min_lat;
   double m_max_lon;
   double m_max_lat;
   double m_pts_grid_width;
   double m_pts_grid_height;
   double m_pts_grid_spacing;
   // mission status
   bool m_pilot_done;

   //std::vector< std::pair<double, double> > m_sample_points;
   std::unordered_map< size_t, Eigen::VectorXd > m_sample_points_unvisited;
   std::unordered_map< size_t, Eigen::VectorXd > m_sample_points_visited;

   // GP, and create mutex for protection of parts of code accessing m_gp
   libgp::GaussianProcess m_gp;
   std::mutex m_gp_mutex;

   // hyperparam optimization in multi-threading
   std::future<bool> m_future_hp_optim;
   bool m_hp_optim_running;
   bool m_hp_optim_done;
};

#endif 
