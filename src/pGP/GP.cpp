/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: GP.cpp                                               */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*                                                               */
/*****************************************************************/

#include "GP.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

#include <boost/lexical_cast.hpp>

#include <cov.h>
#include <cov_se_iso.h>

// check running time
#include <ctime>

// GPLib Rprop
#include "rprop.h"

//---------------------------------------------------------
// Constructor
//
GP::GP() :
  m_gp(2, "CovSum(CovSEiso, CovNoise)"),
  m_hp_optim_running(false),
  m_hp_optim_done(false),
  m_pilot_done(false),
  m_data_added(false),
  m_input_var_data(""),
  m_input_var_sample_points(""),
  m_input_var_sample_points_specs(""),
  m_input_var_pilot_done(""),
  m_input_var_adaptive_trigger(""),
  m_output_var_pred(""),
  m_prediction_interval(-1),
  m_lat(0),
  m_lon(0),
  m_dep(0),
  m_last_published(std::numeric_limits<double>::max()),
  m_need_nxt_wpt(false)
{
  // class variable instantiations can go here
  // as much as possible as function level initialization

  // above we initialize a GP for 2D input data, //TODO convert to 3D
  // using the squared exponential covariance function,
  // with additive white noise

  // Set log-hyperparameter of the covariance function
  Eigen::VectorXd params(m_gp.covf().get_param_dim());
  // hyperparameters: length scale l^2, signal variance s_f^2, noise variance s_n^2
  // note, these can be optimized using cg or rprop
  params << 1.0, 1.0, 1.0; //0.1, 0.1; //-1.6, 3.6, 1.23; -7.52, 3.79, 1.05;
  m_gp.covf().set_loghyper(params);
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB, 
// there is 'new mail', check to see if
// there is anything for this process.
//
bool GP::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key   = msg.GetKey();
    std::string sval  = msg.GetString();
    // separate way for getting the double val (sval was not working for DB_UPTIME)
    double dval  = msg.GetDouble();

#if 0 // Keep these around just for template
    std::string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if ( key == m_input_var_data )
    {
      // add data to GP, coming from uSimBioSensor
      handleMailData(dval);
    }
    else if ( key == "NAV_LAT" )
      m_lat = dval;
    else if ( key == "NAV_LONG" )
      m_lon = dval;
    else if ( key == "NAV_DEPTH" )
      m_dep = dval;
    else if ( key == m_input_var_sample_points )
    {
      // store list of sample locations coming pSamplePoints
      storeSamplePoints(sval);
    }
    else if ( key == m_input_var_sample_points_specs )
    {
      // store specs for sample points
      storeSamplePointsSpecs(sval);
    }
    else if ( key == m_input_var_pilot_done )
    {
      std::cout << "received " << m_input_var_pilot_done << ": " << sval << std::endl;
      m_pilot_done = ( sval == "true" ) ? true : false;
    }
    else if ( key == m_input_var_adaptive_trigger )
    {
      // these are flags set by the wpt behavior
      // if we get it, it must mean a wpt/cycle was done
      m_need_nxt_wpt = true;
    }
    else
      std::cout << "pGP :: Unhandled Mail: " << key << std::endl;
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer
//
bool GP::OnConnectToServer()
{
//  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second
//
bool GP::Iterate()
{
  if ( m_lon == 0 && m_lat == 0 && m_dep == 0 )
    return true;
  else
  {
    // if we received new data,
    // then we have to change the visited/unvisited sample sets
    if ( m_data_added )
    {
      // update sample sets
      updateVisitedSet();
    }

    // when pilot is done,
    // we want to optimize the hyperparams of the GP
    if ( m_pilot_done && !m_hp_optim_done)
    {
      if ( !m_hp_optim_running )
      {
        // start hyperparameter optimization
        m_hp_optim_running = true;

        // start thread for hyperparameter optimization,
        // because this will take a while..
        std::cout << "Starting hyperparameter optimization, current size GP: " << m_gp.get_sampleset_size() << std::endl;
        m_future_hp_optim = std::async(std::launch::async, &GP::runHPOptimization, this, std::ref(m_gp));
      }
      else
      {
        // check if the thread is done
        if ( m_future_hp_optim.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready )
        {
          m_hp_optim_done = m_future_hp_optim.get(); // should be true
          if ( !m_hp_optim_done )
            std::cout << "ERROR: should be done with HP optimization, but get() returns false!" << std::endl;
          m_hp_optim_running = false;
          std::cout << "Done with hyperparameter optimization. New HPs: " << m_gp.covf().get_loghyper() << std::endl;
        }
      }
    }

    // when hyperparameter optimization is done,
    // we want to run adaptive; find next sample locations
    if ( m_hp_optim_done )
    {
      // predict target value and variance for sample locations
      //    if ( (size_t)std::floor(MOOSTime()) % m_prediction_interval == 0  &&  ) // every 5 min, for now
      if ( m_need_nxt_wpt && (std::abs(m_last_published - MOOSTime()) > 1) )
      {
        std::clock_t begin = std::clock();
        findNextSampleLocation();
        std::clock_t end = std::clock();
        std::cout << "runtime findNextSampleLocation: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << '\n' << std::endl;
      }

      if ( m_need_nxt_wpt && (std::abs(m_last_published - MOOSTime()) > 1) )
      {
        std::cout << "resetting nxt wpt need" << std::endl;
        m_need_nxt_wpt = false;
      }

      // periodically, store all GP predictions
      //TODO
    }
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open
bool GP::OnStartUp()
{
  CMOOSApp::OnStartUp();
  
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();

  STRING_LIST::iterator p;
  for (p=sParams.begin(); p!=sParams.end(); p++)
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = true;
    if ( param == "input_var_data" )
    {
      m_input_var_data = toupper(value);
    }
    else if ( param == "input_var_sample_points" )
    {
      m_input_var_sample_points = toupper(value);
    }
    else if ( param == "input_var_sample_points_specs")
    {
      m_input_var_sample_points_specs = toupper(value);
    }
    else if ( param == "input_var_pilot_done" )
    {
      // PILOT_SURVEY_DONE
      m_input_var_pilot_done = toupper(value);
    }
    else if ( param == "input_var_adaptive_trigger" )
    {
      m_input_var_adaptive_trigger = toupper(value);
    }
    else if ( param == "output_var_predictions" )
    {
      m_output_var_pred = toupper(value);
    }
    else if ( param == "prediction_interval" )
    {
      m_prediction_interval = (size_t)atoi(value.c_str());
      if ( m_prediction_interval < 0 )
      {
        std::cout << GetAppName() << " :: ERROR, invalid prediction interval, needs to be > 0" << std::endl;
        RequestQuit();
      }
    }
    else
      handled = false;

    if ( !handled )
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
  }

  if ( m_input_var_data == "" || m_input_var_sample_points == "" )
  {
    std::cout << GetAppName() << " :: ERROR, missing input variable name, exiting." << std::endl;
    RequestQuit();
  }
  else if ( m_output_var_pred == "" )
  {
    std::cout << GetAppName() << " :: ERROR, missing output variable name, exiting." << std::endl;
    RequestQuit();
  }

  registerVariables();

  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables
//            at startup, let the MOOSDB know what you want
//            to receive
//
void GP::registerVariables()
{
  // get vehicle location
  m_Comms.Register("NAV_LAT", 0);
  m_Comms.Register("NAV_LONG", 0);
  m_Comms.Register("NAV_DEPTH", 0);

  // get data for GP
  m_Comms.Register(m_input_var_data, 0);

  // get sample points for GP
  m_Comms.Register(m_input_var_sample_points, 0);
  m_Comms.Register(m_input_var_sample_points_specs, 0);

  // get status mission
  m_Comms.Register(m_input_var_pilot_done, 0);

  // get when wpt cycle finished
  // (when to run adaptive predictions in adaptive state)
  m_Comms.Register(m_input_var_adaptive_trigger, 0);
}

//---------------------------------------------------------
// Procedure: handleMailData
//            handle the incoming message
//
void GP::handleMailData(double received_data)
{
  if ( m_lon == 0 && m_lat == 0 && m_dep == 0 )
    std::cout << GetAppName() << "No NAV_LAT/LON/DEPTH received, not processing data." << std::endl;
  else if ( !m_hp_optim_running )
  {
    // add training data (if not running hyperparameter optimization)
    // Input vectors x must be provided as double[] and targets y as double.
    double x[] = {m_lon, m_lat}; //, m_dep};

//    addPatternToGP(x, received_data);
    // try with threading
    std::thread ap_thread(&GP::addPatternToGP, this, x, received_data);
    ap_thread.detach();

    m_data_added = true;
  }
}

void GP::addPatternToGP(double location[], double value)
{
  // limit scope mutex, protect when adding data
  // because this is now happening in a detached thread
  std::unique_lock<std::mutex> ap_lock(m_gp_mutex);
  if ( ap_lock )
  {
    // log GP: take log (ln) of measurement
    double log_val = log(value);
    m_gp.add_pattern(location, log_val);
    // release mutex
    ap_lock.unlock();
  }
  else
    std::cout << "could not get lock to add: " << location[0] << ","
              << location[1] << "," << value << std::endl;
}

//---------------------------------------------------------
// Procedure: storeSamplePoints
//            parse the string, store the sample locations
//
void GP::storeSamplePoints(std::string input_string)
{
  // input: semicolon separated string of comma separated locations
  // separate by semicolon
  std::vector<std::string> sample_points = parseString(input_string, ';');
  // for each, add to vector
  for ( size_t id_pt = 0; id_pt < sample_points.size(); id_pt++ )
  {
    std::string location = sample_points.at(id_pt);
    size_t comma_pos = location.find(',');
    double lon = (double)atof(location.substr(0,comma_pos).c_str());
    double lat = (double)atof(location.substr(comma_pos+1,location.length()).c_str());

    // new storage:
    // 1. assume ordered as per creation (if needed, we can recalculate the v_id)
    // 2. store in a map; v_id is key, location is value
    // 3. later on, we can easily retrieve from map / store in other
    Eigen::Vector2d loc_vec;
    loc_vec(0) = lon;
    loc_vec(1) = lat;
    m_sample_points_unvisited.insert( std::pair<size_t, Eigen::Vector2d>(id_pt, loc_vec) );

    // the first location should be bottom left corner, store as minima
    if ( id_pt == 0 )
    {
      m_min_lon = lon;
      m_min_lat = lat;
      std::cout << GetAppName() << " :: SW Sample location: " << std::setprecision(10) << lon << " " << lat << std::endl;
    }
    if ( id_pt == sample_points.size()-1 )
    {
      // last location = top right corner (?)
      m_max_lon = lon;
      m_max_lat = lat;
      std::cout << GetAppName() << " :: NE Sample location: " << std::setprecision(10) << lon << " " << lat << std::endl;
    }
  }
  // check / communicate what we did
  std::cout << GetAppName() << " :: stored " << m_sample_points_unvisited.size() << " sample locations" << std::endl;
}

//---------------------------------------------------------
// Procedure: storeSamplePointsSpecs
//            parse the string, store specs for sample locations
//
void GP::storeSamplePointsSpecs(std::string input_string)
{
  // input: comma-separated list of param=value pairs
  // separate by comma
  std::vector<std::string> sample_points_specs = parseString(input_string, ',');
  // for each, check what it is, and store
  for ( size_t param_idx = 0; param_idx < sample_points_specs.size(); param_idx++ )
  {
    std::string param = biteStringX(sample_points_specs[param_idx], '=');
    double value = (double)atof(sample_points_specs[param_idx].c_str());
    if ( param == "width" )
      m_pts_grid_width = value;
    else if ( param == "height" )
      m_pts_grid_height = value;
    else if ( param == "lane_width")
      m_pts_grid_spacing = value;
    else
      std::cout << GetAppName() << " :: error, unhandled part of sample points specs: " << param << std::endl;
  }
  std::cout << GetAppName() << " :: width, height, spacing: " << m_pts_grid_width << ", " <<
               m_pts_grid_height << ", " << m_pts_grid_spacing << std::endl;
}

//---------------------------------------------------------
// Procedure: updateVisitedSet
//            given current vehicle location, move locations
//            from unvisited to visited set
//
void GP::updateVisitedSet()
{
  // handling data, reset state var
  m_data_added = false;

  // store values in tmp to avoid change while this procedure is run
  double veh_lon = m_lon;
  double veh_lat = m_lat;

  // TODO move this to library?
  double lat_deg_to_m = 110923.99118801417;
  double lon_deg_to_m = 92287.20804979937;
  double lon_spacing = m_pts_grid_spacing/lon_deg_to_m; // convert lane width to lon
  double lat_spacing = m_pts_grid_spacing/lat_deg_to_m; // convert lane width to lat

  // if just outside data grid, but close enough,
  // should be able to map to border points
  // buffer of 2 meters
  double buffer_lon = 2/lon_deg_to_m;
  double buffer_lat = 2/lat_deg_to_m;

  if ( veh_lon >= (m_min_lon-buffer_lon) && veh_lat >= (m_min_lat-buffer_lat) &&
       veh_lon <= (m_max_lon+buffer_lon) && veh_lat <= (m_max_lat+buffer_lat) )
  {
    // calculate the id of the location where the vehicle is currently
    // at, and move it to visited
    double x_cell = (veh_lon - m_min_lon)/lon_spacing;
    double y_cell = (veh_lat - m_min_lat)/lat_spacing;
    size_t x_cell_rnd = (size_t) round(x_cell);
    size_t y_cell_rnd = (size_t) round(y_cell);
    size_t y_resolution = (size_t) round(m_pts_grid_height/m_pts_grid_spacing);

    // add one because of zero indexing
    y_resolution++;
    // calculate index into map (stored from SW, y first, then x)
    size_t index = y_resolution*x_cell_rnd + y_cell_rnd;

    std::unordered_map<size_t, Eigen::Vector2d>::iterator curr_loc_itr = m_sample_points_unvisited.find(index);
    if ( curr_loc_itr != m_sample_points_unvisited.end() )
    {
      // remove point from unvisited set
      Eigen::Vector2d move_pt = m_sample_points_unvisited.at(index);

      // check if the sampled point was nearby, if not, there's something wrong
      checkDistanceToSampledPoint(veh_lon, veh_lat, lat_deg_to_m, lon_deg_to_m, move_pt);

      // add the point to the visited set
      m_sample_points_visited.insert(std::pair<size_t, Eigen::Vector2d>(index, move_pt));

      // and remove it from the unvisited set
      m_sample_points_unvisited.erase(curr_loc_itr);

      // report
      std::cout << "\nmoved pt: " << std::setprecision(10) << move_pt(0) << ", " << move_pt(1);
      std::cout << " from unvisited to visited.\n";
      std::cout << "Unvisited size: " << m_sample_points_unvisited.size() << '\n';
      std::cout << "Visited size: " << m_sample_points_visited.size() << '\n' << std::endl;
    }
  }
  else
    return;
}

//---------------------------------------------------------
// Procedure: checkDistanceToSampledPoint
//            we check if, after conversion, the distance
//            of sampled point to vehicle location is reasonable
//
void GP::checkDistanceToSampledPoint(double veh_lon, double veh_lat, double lat_deg_to_m, double lon_deg_to_m, Eigen::Vector2d move_pt)
{
  double dist_lon = std::abs(move_pt(0) - veh_lon);
  double dist_lat = std::abs(move_pt(1)- veh_lat);
  double dist_lon_m = dist_lon*lon_deg_to_m;
  double dist_lat_m = dist_lat*lat_deg_to_m;

  if ( dist_lon_m > m_pts_grid_spacing || dist_lat_m > m_pts_grid_spacing )
  {
    std::cout << GetAppName() << " :: ERROR: distance to chosen sample point is bigger than the grid spacing\n";
    std::cout << GetAppName() << " :: Distance to chosen point: " << dist_lon_m << " " << dist_lat_m << '\n';
    std::cout << GetAppName() << " :: Quitting in 2 seconds." << std::endl;
    sleep(2);
    RequestQuit();
  }
}

//---------------------------------------------------------
// Procedure: findNextSampleLocation
//            find next sample location, using mutual information
//
void GP::findNextSampleLocation()
{
  // predict target value for given input, f()
  // predict variance of prediction for given input, var()

  // for mutual information, we use visited and unvisited sets
  // we want to calculate, for each possible location y,
  // the values of k(y,y), k(y,a), k(a,a), k(y,a*), k(a*,a*)
  // i.e. we calculate the covariance with all the points in the visite
  // and unvisited sets
  // then we calculate sigma^2 (kyy - kya kaa kay) for each set
  // and divide sigma_y_visited by sigma_y_unvisited

  // get covariance function from GP
  // so we can use the get() function from the CovarianceFunction
  // use unique_lock here, such that we can release mutex after m_gp operation
  std::unique_lock<std::mutex> fnsl_lock(m_gp_mutex, std::defer_lock);
  while ( !fnsl_lock.try_lock() ) {}
//    std::cout << "trying to get lock for findNextSampleLocation" << std::endl;
//  }
  checkGPHasData();
  libgp::CovarianceFunction & cov_f = m_gp.covf();
  fnsl_lock.unlock();

  // for each y (from unvisited set only, as in greedy algorithm Krause'08)
  // calculate the mutual information term
  size_t size_visited = m_sample_points_visited.size();
  size_t size_unvisited = m_sample_points_unvisited.size();

  if ( size_visited > 0 )
  {
    // calculate covariance matrices sets, and their inverses (costly operations)
    std::cout << "Calculate covariance matrices" << std::endl;

//    Eigen::MatrixXd K_aa(size_visited, size_visited);
//    createCovarMatrix(cov_f, "visited", K_aa);
//    Eigen::MatrixXd K_aa_inv = K_aa.inverse();

    Eigen::MatrixXd K_avav(size_unvisited, size_unvisited);
    createCovarMatrix(cov_f, "unvisited", K_avav);
    Eigen::MatrixXd K_avav_inv = K_avav.inverse();
    // TODO replace inverses using Cholesky decomposition

    // construct a vector of target values for the unvisited locations
    // using the GP
    std::cout << "Get target values for unvisited points" << std::endl;
    Eigen::VectorXd t_av(size_unvisited);
    getTgtValUnvisited(t_av);

    std::cout << "Calculate MI" << std::endl;
    m_future_next_pt = std::async(std::launch::async, &GP::calcMICriterion, this, t_av, std::ref(cov_f), K_avav_inv);
//    calcMICriterion(t_av, cov_f, K_avav_inv, size_unvisited, best_so_far_y, best_so_far);

    // publish greedy best
    publishNextBestPosition(m_future_next_pt.get());
  }
}

//---------------------------------------------------------
// Procedure: publishNextBestPosition
//            call Notify & publish location
//
void GP::publishNextBestPosition(Eigen::Vector2d best_so_far_y)
{
  // app feedback
  std::cout << GetAppName() << " :: publishing " << m_output_var_pred << '\n';
  std::cout << GetAppName() << " :: current best next y: " << std::setprecision(15) << best_so_far_y(0) << ", " << best_so_far_y(1) << '\n';

  std::ostringstream output_stream;
  output_stream << std::setprecision(15) << best_so_far_y(0) << "," << best_so_far_y(1);
  m_Comms.Notify(m_output_var_pred, output_stream.str());

  // update state vars
  m_last_published = MOOSTime();
  m_need_nxt_wpt = false;
}

//---------------------------------------------------------
// Procedure: calcMICriterion
//            do the MI criterion calculation and find best y
//
Eigen::Vector2d GP::calcMICriterion(Eigen::VectorXd t_av, libgp::CovarianceFunction& cov_f, Eigen::MatrixXd K_avav_inv)
//                         , size_t size_unvisited, Eigen::Vector2d & best_so_far_y, double & best_so_far)
{
  size_t size_unvisited = m_sample_points_unvisited.size();
  double best_so_far = -1*std::numeric_limits<double>::max();
  Eigen::Vector2d best_so_far_y(2);

  std::unordered_map<size_t, Eigen::Vector2d>::iterator y_itr;
  size_t best_cnt = 1;
  for ( y_itr = m_sample_points_unvisited.begin(); y_itr != m_sample_points_unvisited.end(); y_itr++ )
  {
    Eigen::Vector2d y = y_itr->second;

    // calculate k(y,y)
    double k_yy = cov_f.get(y,y);

    // calculate covariance with visited set
//      Eigen::VectorXd k_ya(size_visited);
//      createCovarVector(cov_f, y, "visited", k_ya);
//      // TODO: add noise term?
//      double mat_ops_result = k_ya.transpose() * K_aa_inv * k_ya;
//      double sigma_y_A = k_yy - mat_ops_result;

    // covariance with visited set should be the predictive covariance
    // from the GP
    double y_loc[2] = {y(0), y(1)};
    double pred_mean_yA = m_gp.f(y_loc);
    double pred_cov_yA = m_gp.var(y_loc);

    // calculate covariance with unvisited set
    Eigen::VectorXd k_yav(size_unvisited);
    createCovarVector(cov_f, y, "unvisited", k_yav);
    // TODO add noise term in both equations! TODO TODO
    double mat_ops_result = k_yav.transpose() * K_avav_inv * k_yav;
    double sigma_y_Av = k_yy - mat_ops_result;

    // predictive mean of unvisited set -- TODO check?
    double pred_mean_yAv = k_yav.transpose() * K_avav_inv * t_av;

    // convert to log GP
    double mean_yA_lGP, var_yA_lGP; // visited set
    logGPfromGP(pred_mean_yA, pred_cov_yA, mean_yA_lGP, var_yA_lGP);
    double mean_yAv_lGP, var_yAv_lGP; // unvisited set
    logGPfromGP(pred_mean_yAv, sigma_y_Av, mean_yAv_lGP, var_yAv_lGP);

    // calculate mutual information term
//      double div = 0.5 * log(sigma_y_A / sigma_y_Av);
    double div = 0.5 * log( var_yA_lGP / var_yAv_lGP );

    // store max (greedy best)
    if ( div > best_so_far )
    {
      best_so_far = div;
      best_so_far_y = y;
      best_cnt++;
    }
  }
  std::cout << GetAppName() << " :: best_cnt: " << best_cnt << std::endl;
  // TODO: store all values, return sorted list?
  return best_so_far_y;
}

//---------------------------------------------------------
// Procedure: getTgtValUnvisited
//            construct vector with target values for
//            unvisited points, as extracted from GP
//
void GP::getTgtValUnvisited(Eigen::VectorXd & t_av)
{
  std::unique_lock<std::mutex> tgt_val_lock(m_gp_mutex, std::defer_lock);
  std::unordered_map<size_t, Eigen::Vector2d>::iterator av_itr;
  size_t t_cnt = 0;
  // use unique_lock here, such that we can release mutex after m_gp operation
  while ( !tgt_val_lock.try_lock() ) {}
//    std::cout << "trying to get lock for t_av calculation" << std::endl;
  for ( av_itr = m_sample_points_unvisited.begin(); av_itr != m_sample_points_unvisited.end(); av_itr++, t_cnt++ )
  {
    double av_loc[2] = {av_itr->second(0), av_itr->second(1)};
    t_av(t_cnt) = m_gp.f(av_loc);
  }
  tgt_val_lock.unlock();
}

//---------------------------------------------------------
// Procedure: checkGPHasData
//            check that the GP has been filled, else quit
//
void GP::checkGPHasData()
{
  if ( m_gp.get_sampleset_size() == 0 )
  {
    std::cout << GetAppName() << " :: ERROR, trying to predict without data. Exiting in 2 seconds" << std::endl;
    sleep(2);
    RequestQuit();
  }
}

//---------------------------------------------------------
// Procedure: createCovarVector
//            helper method for mutual information calculation
//            calculates K_ya
//
void GP::createCovarVector(libgp::CovarianceFunction& cov_f, Eigen::Vector2d y, std::string const & set_identifier, Eigen::VectorXd & k_ya)
{
  // choose which map to use
  std::unordered_map<size_t, Eigen::Vector2d> & map_ref = ( set_identifier == "visited" ? m_sample_points_visited : m_sample_points_unvisited);

  // calculate the covariances
  std::unordered_map<size_t, Eigen::Vector2d>::iterator a_itr;
  size_t a_cnt = 0;
  for ( a_itr = map_ref.begin(); a_itr != map_ref.end(); a_itr++, a_cnt++)
  {
    // calc k_ya
    Eigen::Vector2d a = a_itr->second;

    k_ya(a_cnt) = cov_f.get(y,a);
  }
}

//---------------------------------------------------------
// Procedure: createCovarMatrix
//            helper method for mutual information calculation
//            calculates K_AA
//
void GP::createCovarMatrix(libgp::CovarianceFunction& cov_f, std::string const & set_identifier, Eigen::MatrixXd & K_aa)
{
  // choose which map to use
  std::unordered_map<size_t, Eigen::Vector2d> & map_ref = ( set_identifier == "visited" ? m_sample_points_visited : m_sample_points_unvisited);

  std::unordered_map<size_t, Eigen::Vector2d>::iterator a_itr;
  size_t a_cnt = 0;
  for ( a_itr = map_ref.begin(); a_itr != map_ref.end(); a_itr++, a_cnt++)
  {
    Eigen::Vector2d a = a_itr->second;

    // calc K_aa
    std::unordered_map<size_t, Eigen::Vector2d>::iterator b_itr;
    size_t b_cnt = a_cnt;
    // iterate only over triangle of matrix to reduce computation
    for ( b_itr = a_itr; b_itr != map_ref.end(); b_itr++, b_cnt++ )
    {
      Eigen::Vector2d b = b_itr->second;

      double current_cov_val = cov_f.get(a, b);
      K_aa(a_cnt, b_cnt) = current_cov_val;
      if (a_cnt != b_cnt)
        K_aa(b_cnt, a_cnt) = current_cov_val;
    }
  }
}

//---------------------------------------------------------
// Procedure: runHPOptimization()
//            run in thread, call GP's hyperparam optimization
//
bool GP::runHPOptimization(libgp::GaussianProcess & gp)
{
  // protect GP access with mutex
  std::unique_lock<std::mutex> hp_lock(m_gp_mutex, std::defer_lock);
  while ( !hp_lock.try_lock() ){}
//  {
//    std::cout << "trying to get lock for HP optim" << std::endl;
//  }
  std::cout << "obtained lock, continuing HP optimization" << std::endl;

  std::clock_t begin = std::clock();

  // optimization
  // there are 2 methods in gplib, conjugate gradient and RProp, the latter 
  // should be more efficient
  libgp::RProp rprop;
  rprop.init();

  // RProp arguments: GP, 'n' (nr iterations), verbose
  rprop.maximize(&gp, 10, 0);

  std::clock_t end = std::clock();
  std::cout << "runtime hyperparam optimization: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  hp_lock.unlock();

  return true;
}

void GP::logGPfromGP(double gp_mean, double gp_cov, double & lgp_mean, double & lgp_cov )
{
  // convert GP mean to log GP mean (lognormal mean)
  lgp_mean = exp(gp_mean + gp_cov/2.0);
  lgp_cov = (lgp_mean*lgp_mean) * (exp(gp_cov) - 1.0);
}
