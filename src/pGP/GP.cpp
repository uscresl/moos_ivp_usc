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

// write to file
#include <fstream>

//---------------------------------------------------------
// Constructor
//
GP::GP() :
  m_input_var_data(""),
  m_input_var_sample_points(""),
  m_input_var_sample_points_specs(""),
  m_input_var_adaptive_trigger(""),
  m_output_var_pred(""),
  m_output_filename_prefix(""),
  m_prediction_interval(-1),
  m_use_MI(false),
  m_use_log_gp(true),
  m_lat(0),
  m_lon(0),
  m_dep(0),
  m_pause_data_adding(false),
  m_last_published(std::numeric_limits<double>::max()),
  m_last_pred_save(std::numeric_limits<double>::max()),
  m_min_lon(0.0),
  m_min_lat(0.0),
  m_max_lon(0.0),
  m_max_lat(0.0),
  m_pts_grid_width(0.0),
  m_pts_grid_height(0.0),
  m_pts_grid_spacing(0.0),
  m_lon_spacing(0.0),
  m_lat_spacing(0.0),
  m_buffer_lon(0.0),
  m_buffer_lat(0.0),
  m_y_resolution(0.0),
  m_lon_deg_to_m(0.0),
  m_lat_deg_to_m(0.0),
  m_pilot_done_time(0.0),
  m_need_nxt_wpt(false),
  m_finding_nxt(false),
  m_gp(2, "CovSum(CovSEiso, CovNoise)"),
  m_hp_optim_running(false),
  m_hp_optim_done(false),
  m_data_mail_counter(1),
  m_finished(false),
  m_hp_optim_mode_cnt(0)
{
  // class variable instantiations can go here
  // as much as possible as function level initialization

  // TODO move this to library?
  m_lat_deg_to_m = 110923.99118801417;
  m_lon_deg_to_m = 92287.20804979937;

  // above we initialize a GP for 2D input data, //TODO convert to 3D
  // using the squared exponential covariance function,
  // with additive white noise

  // Set (log of) hyperparameters of the covariance function
  Eigen::VectorXd params(m_gp.covf().get_param_dim());
  // hyperparameters: length scale l^2, signal variance s_f^2, noise variance s_n^2
  // note, these will be optimized using cg or rprop
  // length scale: avg lat/lon_deg_to_m is 10000, 10m range = 0.001
  //               0.002^2=0.000004, ln() = -12.4292
  // signal: from 0 to ca 30, log scale <0 to 1.5
  //         stdev 1.5, ln() = 0.4055
  // noise: let's set 10x smaller than signal
  //        stdev 0.15, ln() = -1.8971
  params << -12.4292, 0.4055, -1.8971;
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
      // note, if we are circling at location to calculate next waypoint,
      // then it makes no sense to keep adding those data points, so
      // let's not do that
      // presumably we will have added the location just before we reached
      // the waypoint (if not, maybe change this to add once)
      // similarly for when we are calculating the hyperparameters
      // although this is outside the area so should not be a problem anyway
      m_data_mail_counter++;
      if ( !m_pause_data_adding && !m_hp_optim_running && (m_data_mail_counter % 2 == 0) )
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
      // (done once)
      storeSamplePoints(sval);
    }
    else if ( key == m_input_var_sample_points_specs )
    {
      // store specs for sample points
      // (done once)
      storeSamplePointsSpecs(sval);
    }
    else if ( key == m_input_var_adaptive_trigger )
    {
      // check when adaptive waypoint reached
      // these are flags set by the adaptive wpt behavior
      // if we get it, it must mean a wpt/cycle was done
      m_need_nxt_wpt = true;
    }
    else if ( key == "MODE" )
    {
      // check the mission state for whether we need to do HP optimization
      if ( sval.find("HP_OPTIM") != std::string::npos )
      {
        m_hp_optim_mode_cnt++;
        if ( m_hp_optim_mode_cnt == 1 )
          m_pilot_done_time = MOOSTime();
      }
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
    // when pilot is done,
    // we want to optimize the hyperparams of the GP
    if ( m_hp_optim_mode_cnt == 1 && !m_hp_optim_done && !m_finished )
    {
      if ( !m_hp_optim_running )
      {
        // start hyperparameter optimization
        m_hp_optim_running = true;

        // start thread for hyperparameter optimization,
        // because this will take a while..
        std::cout << "Starting hyperparameter optimization, current size GP: " << m_gp.get_sampleset_size() << std::endl;
        m_future_hp_optim = std::async(std::launch::async, &GP::runHPOptimization, this, std::ref(m_gp), 20);
      }
      else
      {
        // running HP optimization
        // check if the thread is done
        if ( m_future_hp_optim.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
        {
          m_hp_optim_done = m_future_hp_optim.get(); // should be true
          if ( !m_hp_optim_done )
            std::cout << "ERROR: should be done with HP optimization, but get() returns false!" << std::endl;
          m_hp_optim_running = false;
          std::cout << "Done with hyperparameter optimization. New HPs: " << m_gp.covf().get_loghyper() << std::endl;
          m_Comms.Notify("STAGE","survey");
        }
      }
    }

    // when hyperparameter optimization is done,
    // we want to run adaptive; find next sample locations
    if ( m_hp_optim_done && m_hp_optim_mode_cnt < 2 && !m_finished)
    {
      // predict target value and variance for sample locations
      //    if ( (size_t)std::floor(MOOSTime()) % m_prediction_interval == 0  &&  ) // every 5 min, for now
      if ( m_need_nxt_wpt && (std::abs(m_last_published - MOOSTime()) > 1.0) )
      {
        if ( !m_finding_nxt )
        {
          std::cout << "calling to find next sample location" << std::endl;
          findNextSampleLocation();
        }
        else
        {
          m_pause_data_adding = true;
          // see if we can get result from future
          if ( m_future_next_pt.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
          {
            m_finding_nxt = false;
            // publish greedy best
            publishNextBestPosition(); //m_future_next_pt.get());
            m_pause_data_adding = false;
          }
        }
      }

      // periodically (every 300s = 5min), store all GP predictions
      // only after pilot done, first 300 seconds after pilot done time
      // make sure we store the GP right after HP optimization for comparison
      if ( m_hp_optim_done && (std::abs(m_last_pred_save - MOOSTime()) > 1.0 ) &&
           ( (MOOSTime()-m_pilot_done_time < 2) ||
             ((size_t)std::floor(MOOSTime()-m_pilot_done_time) % 300 == 0) ) )
      {
        std::thread pred_store(&GP::makeAndStorePredictions, this);
        pred_store.detach();
        m_last_pred_save = MOOSTime();
      }
    }

    // when returning, do a final HP optimization
    if ( m_hp_optim_mode_cnt == 2 && !m_finished )
    {
      if ( !m_hp_optim_running )
      {
        // start hyperparameter optimization
        m_hp_optim_running = true;

        // start thread for hyperparameter optimization,
        // because this will take a while..
        std::cout << "Starting hyperparameter optimization, current size GP: " << m_gp.get_sampleset_size() << std::endl;
        m_future_hp_optim = std::async(std::launch::async, &GP::runHPOptimization, this, std::ref(m_gp), 10);
      }
      else
      {
        // running HP optimization
        // check if the thread is done
        if ( m_future_hp_optim.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
        {
          m_hp_optim_done = m_future_hp_optim.get(); // should be true
          if ( !m_hp_optim_done )
            std::cout << "ERROR: should be done with HP optimization, but get() returns false!" << std::endl;
          m_hp_optim_running = false;
          std::cout << "Done with hyperparameter optimization. New HPs: " << m_gp.covf().get_loghyper() << std::endl;
          m_Comms.Notify("STAGE","return");

          // store predictions one last time
          std::thread pred_store(&GP::makeAndStorePredictions, this);
          pred_store.detach();

          m_finished = true;
        }
      }
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
    else if ( param == "output_filename_prefix" )
    {
      m_output_filename_prefix = value;
    }
    else if ( param == "use_log_gp" )
    {
      m_use_log_gp = (value == "true" ? true : false);
      std::cout << GetAppName() << " :: using log GP? " << (m_use_log_gp ? "yes" : "no") << std::endl;
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

  std::thread data_thread(&GP::dataAddingThread, this);
  data_thread.detach();

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
  m_Comms.Register("MODE", 0);

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
  else
  {
    // if not running hyperparameter optimization or calculating nxt wpt
    // add training data

    // get the current vehicle location, such that we can pass it on to the functions
    // that add the data and update the (un)visited maps,
    // which can run in parallel
    double veh_lon = m_lon;
    double veh_lat = m_lat;

    // try with threading, because this becomes more costly as GP grows
    // TODO pass into queue and run different function as thread to handle queue?
    std::vector<double> nw_data_pt{veh_lon, veh_lat, received_data};
    m_queue_data_points_for_gp.push(nw_data_pt);

//    std::thread ap_thread(&GP::addPatternToGP, this, received_data, veh_lon, veh_lat);
//    ap_thread.detach();

//    // if we received new data,
//    // then we have to change the visited/unvisited sample sets
//    // thread this call to make sure it does not interfere with the mail cycle

//    // grab lon and lat from location array
//    int index = get_index_for_map(veh_lon, veh_lat);
//    if ( index >= 0 )
//    {
//      bool update_needed = need_to_update_maps((size_t)index);
//      if ( update_needed )
//      {
//        std::thread update_visited(&GP::updateVisitedSet, this, veh_lon, veh_lat, (size_t)index);
//        update_visited.detach();
//      }
//    }
  }
}

void GP::dataAddingThread()
{
  while ( !m_finished )
  {
    if ( m_queue_data_points_for_gp.empty() )
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    else
    {
      // process data point
      std::vector<double> data_pt = m_queue_data_points_for_gp.front();
      m_queue_data_points_for_gp.pop();
      double veh_lon = data_pt[0];
      double veh_lat = data_pt[1];
      // add data pt
      addPatternToGP(veh_lon, veh_lat, data_pt[2]);
      // update visited set if needed
      int index = get_index_for_map(veh_lon, veh_lat);
      if ( index >= 0 && need_to_update_maps((size_t)index) )
        updateVisitedSet(veh_lon, veh_lat, (size_t)index);
    }
  }
}

void GP::addPatternToGP(double veh_lon, double veh_lat, double value)
{
  // limit scope mutex, protect when adding data
  // because this is now happening in a detached thread
  double location[2] = {veh_lon, veh_lat};

  // log GP: take log (ln) of measurement
  double save_val = m_use_log_gp ? log(value) : value;

  // Input vectors x must be provided as double[] and targets y as double.

  std::unique_lock<std::mutex> ap_lock(m_gp_mutex, std::defer_lock);
  // obtain lock
  while ( !ap_lock.try_lock() ) {}
  // add new data point to GP
  m_gp.add_pattern(location, save_val);
  // release mutex
  ap_lock.unlock();
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
  m_sample_locations.reserve(sample_points.size());

  std::ofstream ofstream_loc;
  std::string m_file_loc = (m_output_filename_prefix + "_locations.csv");
  ofstream_loc.open(m_file_loc, std::ios::out);

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
    // copy the points to have all sample points for easier processing for predictions
    m_sample_locations.push_back(std::pair<double, double>(lon, lat));
    // save to file
    ofstream_loc << std::setprecision(15) << lon << ", " << lat << '\n';

    // the first location should be bottom left corner, store as minima
    if ( id_pt == 0 )
    {
      m_min_lon = lon;
      m_min_lat = lat;
      std::cout << GetAppName() << " :: SW Sample location: " << std::setprecision(10) << lon << " " << lat << std::endl;
    }
    if ( id_pt == sample_points.size()-1 )
    {
      // last location = top right corner, store as maxima
      m_max_lon = lon;
      m_max_lat = lat;
      std::cout << GetAppName() << " :: NE Sample location: " << std::setprecision(10) << lon << " " << lat << std::endl;
    }
  }
  ofstream_loc.close();
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
    {
      m_pts_grid_spacing = value;
      calcLonLatSpacingAndBuffers();
    }
    else
      std::cout << GetAppName() << " :: error, unhandled part of sample points specs: " << param << std::endl;
  }
  std::cout << GetAppName() << " :: width, height, spacing: " << m_pts_grid_width << ", " <<
               m_pts_grid_height << ", " << m_pts_grid_spacing << std::endl;
}

//---------------------------------------------------------
// Procedure: need_to_update_maps
//            check if location's grid index is in unvisited map
//
bool GP::need_to_update_maps(size_t grid_index)
{
    // add mutex for changing of global maps
    std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
    while ( !map_lock.try_lock() ){}
    std::unordered_map<size_t, Eigen::Vector2d>::iterator curr_loc_itr = m_sample_points_unvisited.find(grid_index);
    map_lock.unlock();

    return ( curr_loc_itr != m_sample_points_unvisited.end() );
}

//---------------------------------------------------------
// Procedure: get_index_for_map
//            calculate the grid index for the vehicle location
//
int GP::get_index_for_map(double veh_lon, double veh_lat)
{
  if ( veh_lon >= (m_min_lon-m_buffer_lon) && veh_lat >= (m_min_lat-m_buffer_lat) &&
       veh_lon <= (m_max_lon+m_buffer_lon) && veh_lat <= (m_max_lat+m_buffer_lat) )
  {
    // calculate the id of the location where the vehicle is currently
    // at, and move it to visited
    size_t x_cell_rnd = (size_t) round((veh_lon - m_min_lon)/m_lon_spacing);
    size_t y_cell_rnd = (size_t) round((veh_lat - m_min_lat)/m_lat_spacing);

    // calculate index into map (stored from SW, y first, then x)
    int index = m_y_resolution*x_cell_rnd + y_cell_rnd;

    return index;
  }
  return -1;
}

//---------------------------------------------------------
// Procedure: updateVisitedSet
//            given current vehicle location, move locations
//            from unvisited to visited set
//
void GP::updateVisitedSet(double veh_lon, double veh_lat, size_t index )
{
  // add mutex for changing of global maps
  std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
  while ( !map_lock.try_lock() ){}

  std::unordered_map<size_t, Eigen::Vector2d>::iterator curr_loc_itr = m_sample_points_unvisited.find(index);
  if ( curr_loc_itr != m_sample_points_unvisited.end() )
  {
    // remove point from unvisited set
    Eigen::Vector2d move_pt = curr_loc_itr->second;

    // check if the sampled point was nearby, if not, there's something wrong
    checkDistanceToSampledPoint(veh_lon, veh_lat, move_pt);

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

  map_lock.unlock();
}

//---------------------------------------------------------
// Procedure: checkDistanceToSampledPoint
//            we check if, after conversion, the distance
//            of sampled point to vehicle location is reasonable
//
void GP::checkDistanceToSampledPoint(double veh_lon, double veh_lat, Eigen::Vector2d move_pt)
{
  double dist_lon = std::abs(move_pt(0) - veh_lon);
  double dist_lat = std::abs(move_pt(1) - veh_lat);
  double dist_lon_m = dist_lon*m_lon_deg_to_m;
  double dist_lat_m = dist_lat*m_lat_deg_to_m;

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
  std::cout << "find nxt sample loc" << std::endl;

  if ( checkGPHasData() )
  {
    // for each y (from unvisited set only, as in greedy algorithm Krause'08)
    // calculate the mutual information term
    if ( m_sample_points_visited.size() > 0 )
    {
      m_finding_nxt = true;
      // use threading because this is a costly operation that would otherwise
      // interfere with the MOOS app rate
      if ( m_use_MI )
      {
        // get covariance function from GP
        // such that we can use the get() function from the CovarianceFunction
        libgp::CovarianceFunction & cov_f = m_gp.covf();
        m_future_next_pt = std::async(std::launch::async, &GP::calcMICriterion, this, std::ref(cov_f));
      }
      else
        m_future_next_pt = std::async(std::launch::async, &GP::calcMECriterion, this);
    }
  }
}

//---------------------------------------------------------
// Procedure: publishNextBestPosition
//            call Notify & publish location
//
void GP::publishNextBestPosition() //Eigen::Vector2d best_so_far_y)
{
  Eigen::Vector2d best_so_far_y = m_future_next_pt.get();

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
// Procedure: calcMECriterion
//            calculate maximum entropy
//            for every unvisited location,
//            and pick best (greedy)
//
Eigen::Vector2d GP::calcMECriterion()
{
  std::cout << "max entropy start" << std::endl;

  std::clock_t begin = std::clock();

  Eigen::Vector2d best_so_far_y(2);
  double best_so_far = -1*std::numeric_limits<double>::max();
  std::unordered_map<size_t, Eigen::Vector2d>::iterator y_itr;

  std::cout << "try for lock gp" << std::endl;

  // lock for access to m_gp
  std::unique_lock<std::mutex> gp_lock(m_gp_mutex, std::defer_lock);
  // use unique_lock here, such that we can release mutex after m_gp operation
  while ( !gp_lock.try_lock() ) {}
  std::cout << "make copy GP" << std::endl;
  libgp::GaussianProcess gp_copy(m_gp);
  // release lock
  gp_lock.unlock();

  std::cout << "try for lock map" << std::endl;
  std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
  while ( !map_lock.try_lock() ){}
  // make copy of map to use instead of map,
  // such that we do not have to lock it for long
  std::unordered_map<size_t, Eigen::Vector2d> unvisited_map_copy;
  unvisited_map_copy.insert(m_sample_points_unvisited.begin(), m_sample_points_unvisited.end());
  map_lock.unlock();

  std::cout << "calc max entropy" << std::endl;

  // for each unvisited location
  for ( y_itr = unvisited_map_copy.begin(); y_itr != unvisited_map_copy.end(); y_itr++ )
  {
    // get unvisited location
    Eigen::Vector2d y = y_itr->second;
    double y_loc[2] = {y(0), y(1)};

    // calculate its posterior entropy
    double pred_mean = gp_copy.f(y_loc);
    double pred_cov = gp_copy.var(y_loc);

    // normal distribution
    //  1/2 ln ( 2*pi*e*sigma^2 )
    double post_entropy;
    if ( !m_use_log_gp )
      post_entropy = log( 2 * M_PI * exp(1) * pred_cov);
    else
    {
      // lognormal distribution
      double var_part = (1/2.0) * log( 2 * M_PI * exp(1) * pred_cov);
      post_entropy = var_part + pred_mean;
    }

    if ( post_entropy > best_so_far )
    {
      best_so_far = post_entropy;
      best_so_far_y = y;
    }
  }

  std::clock_t end = std::clock();
  std::cout << "Max Entropy calc time: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  // copy of GP and unvisited_map get destroyed when this function exits
  return best_so_far_y;
}

//---------------------------------------------------------
// Procedure: calcMICriterion
//            do the MI criterion calculation and find best y
//
Eigen::Vector2d GP::calcMICriterion(libgp::CovarianceFunction& cov_f)
{
  // for mutual information, we use visited and unvisited sets
  // we want to calculate, for each possible location y,
  // the values of k(y,y), k(y,a), k(a,a), k(y,a*), k(a*,a*)
  // i.e. we calculate the covariance with all the points in the visite
  // and unvisited sets
  // then we calculate sigma^2 (kyy - kya kaa kay) for each set
  // and divide sigma_y_visited by sigma_y_unvisited

  // calculate covariance matrices sets, and their inverses (costly operations)
  std::cout << "Calculate covariance matrices" << std::endl;

  std::unique_lock<std::mutex> mi_lock(m_gp_mutex, std::defer_lock);
  // use unique_lock here, such that we can release mutex after m_gp operation
  while ( !mi_lock.try_lock() ) {}

  size_t size_unvisited = m_sample_points_unvisited.size();

  // note; replaced with using predictions from GP
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

  double best_so_far = -1*std::numeric_limits<double>::max();
  Eigen::Vector2d best_so_far_y(2);
  std::unordered_map<size_t, Eigen::Vector2d>::iterator y_itr;
  size_t best_cnt = 1;

  std::cout << "calc for all y (" << m_sample_points_unvisited.size() << ")" << std::endl;
  std::clock_t begin = std::clock();
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
    // from the GP (note, GP actually contains more data pts .. but at least
    //              the visited locations)
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

    // calculate mutual information term
    double div;
    if ( m_use_log_gp )
    {
      // convert to log GP
      double mean_yA_lGP, var_yA_lGP; // visited set
      logGPfromGP(pred_mean_yA, pred_cov_yA, mean_yA_lGP, var_yA_lGP);
      double mean_yAv_lGP, var_yAv_lGP; // unvisited set
      logGPfromGP(pred_mean_yAv, sigma_y_Av, mean_yAv_lGP, var_yAv_lGP);

      // TODO change for log GP, incorp mean, current incorrect (is for GP)
      div = 0.5 * log( var_yA_lGP / var_yAv_lGP );
    }
    else
    {
      // double div = 0.5 * log(sigma_y_A / sigma_y_Av);
      div = 0.5 * log ( pred_cov_yA / sigma_y_Av );
    }

    // store max (greedy best)
    if ( div > best_so_far )
    {
      best_so_far = div;
      best_so_far_y = y;
      best_cnt++;
    }
  }
  std::clock_t end = std::clock();
  std::cout << "MI crit calc time: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  std::cout << GetAppName() << " :: best_cnt: " << best_cnt << std::endl;

  mi_lock.unlock();

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
  std::unordered_map<size_t, Eigen::Vector2d>::iterator av_itr;
  size_t t_cnt = 0;

  for ( av_itr = m_sample_points_unvisited.begin(); av_itr != m_sample_points_unvisited.end(); av_itr++, t_cnt++ )
  {
    double av_loc[2] = {av_itr->second(0), av_itr->second(1)};
    t_av(t_cnt) = m_gp.f(av_loc);
  }
}

//---------------------------------------------------------
// Procedure: checkGPHasData
//            check that the GP has been filled, else quit
//
bool GP::checkGPHasData()
{
  return ( m_gp.get_sampleset_size() > 0 );
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
bool GP::runHPOptimization(libgp::GaussianProcess & gp, size_t nr_iterations)
{
  // protect GP access with mutex
  std::unique_lock<std::mutex> hp_lock(m_gp_mutex, std::defer_lock);
  while ( !hp_lock.try_lock() ){}
  std::cout << "obtained lock, continuing HP optimization" << std::endl;

  std::clock_t begin = std::clock();

  // optimization
  // there are 2 methods in gplib, conjugate gradient and RProp, the latter 
  // should be more efficient
  libgp::RProp rprop;
  rprop.init();

  // RProp arguments: GP, 'n' (nr iterations), verbose
  rprop.maximize(&gp, nr_iterations, 0);

  std::clock_t end = std::clock();
  std::cout << "runtime hyperparam optimization: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

//  // test own idea HPs

//  // Set log-hyperparameter of the covariance function
//  Eigen::VectorXd params(m_gp.covf().get_param_dim());
//  // hyperparameters: length scale l^2, signal variance s_f^2, noise variance s_n^2
//  // note, these can be optimized using cg or rprop
//  // length scale: avg lat/lon_deg_to_m is 10000, 10m range = 0.001
//  // signal: from 0 to ca 30, log scale <0 to 1.5
//  // noise: let's set 10x smaller than signal
//  params << 0.000001, 0.01, 0.0001;
//  m_gp.covf().set_loghyper(params);

  // test write to file
  begin = std::clock();
  std::stringstream filenm;
  filenm << "hp_optim_" << nr_iterations;
  gp.write(filenm.str().c_str());
  end = std::clock();
  std::cout << "write to file time: " <<  ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  hp_lock.unlock();

  return true;
}

//---------------------------------------------------------
// Procedure: logGPfromGP
//            convert pred mean/var from GP to pred mean/var for log GP
//
void GP::logGPfromGP(double gp_mean, double gp_cov, double & lgp_mean, double & lgp_cov )
{
  // convert GP mean to log GP mean (lognormal mean)
  lgp_mean = exp(gp_mean + gp_cov/2.0);
  lgp_cov = (lgp_mean*lgp_mean) * (exp(gp_cov) - 1.0);
}

//---------------------------------------------------------
// Procedure: makeAndStorePredictions
//            file writing
//
void GP::makeAndStorePredictions()
{
  // make a copy of the GP and use that below, to limit lock time
  std::unique_lock<std::mutex> write_lock(m_gp_mutex, std::defer_lock);
  while ( !write_lock.try_lock() ) {}
  std::cout << "store predictions" << std::endl;
  libgp::GaussianProcess gp_copy(m_gp);
  write_lock.unlock();

  // prepare file actions
  std::ofstream ofstream_pm, ofstream_pv, ofstream_mu, ofstream_s2;

  if ( m_use_log_gp )
  {
    // params lognormal distr
    std::string m_file_pm = (m_output_filename_prefix + "_pred_mean.csv");
    std::string m_file_pv = (m_output_filename_prefix + "_pred_var.csv");
    ofstream_pm.open(m_file_pm, std::ios::app);
    ofstream_pv.open(m_file_pv, std::ios::app);
  }
  // params normal distr
  std::string m_file_mu = (m_output_filename_prefix + "_pred_mu.csv");
  std::string m_file_s2 = (m_output_filename_prefix + "_pred_sigma2.csv");
  ofstream_mu.open(m_file_mu, std::ios::app);
  ofstream_s2.open(m_file_s2, std::ios::app);

  // make predictions
  std::vector< std::pair<double, double> >::iterator loc_itr;
  bool first = true;

  for ( loc_itr = m_sample_locations.begin(); loc_itr < m_sample_locations.end(); loc_itr++ )
  {
    double loc[2] {loc_itr->first, loc_itr->second};
    double pred_mean; // = gp_copy.f(loc);
    double pred_var; // = gp_copy.var(loc);
    gp_copy.f_and_var(loc, pred_mean, pred_var);

    double pred_mean_lGP, pred_var_lGP;
    if ( m_use_log_gp )
    {
      // convert to lGP mean and variance
      // to get back in the 'correct' scale for comparison to generated data
      logGPfromGP(pred_mean, pred_var, pred_mean_lGP, pred_var_lGP);
    }

    // save to file, storing predictions
    if ( !first )
    {
      if ( m_use_log_gp )
      {
        ofstream_pm << ", ";
        ofstream_pv << ", ";
      }
      ofstream_mu << ", ";
      ofstream_s2 << ", ";
    }
    first = false;
    if ( m_use_log_gp )
    {
      ofstream_pm << pred_mean_lGP;
      ofstream_pv << pred_var_lGP;
    }
    ofstream_mu << pred_mean;
    ofstream_s2 << pred_var;
  }
  if ( m_use_log_gp )
  {
    ofstream_pm << '\n';
    ofstream_pv << '\n';
    ofstream_pm.close();
    ofstream_pv.close();
  }

  ofstream_mu << '\n';
  ofstream_s2 << '\n';
  ofstream_mu.close();
  ofstream_s2.close();

  std::cout << "done saving predictions to file" << std::endl;
  // copy of GP gets destroyed when this function exits
}

//---------------------------------------------------------
// Procedure: calcLonLatSpacingAndBuffers
//            separate out these calculations
//            such that we only do them once
//
void GP::calcLonLatSpacingAndBuffers()
{
  // convert lane widths in meters to lon/lat
  m_lon_spacing = m_pts_grid_spacing/m_lon_deg_to_m;
  m_lat_spacing = m_pts_grid_spacing/m_lat_deg_to_m;

  // calculate the amount of y vertices
  m_y_resolution = (size_t) round(m_pts_grid_height/m_pts_grid_spacing);
  // add one because of zero indexing
  m_y_resolution++;

  // if just outside data grid, but close enough,
  // should be able to map to border points
  // buffer of 2 meters (TODO make parameter?)
  m_buffer_lon = 2.0/m_lon_deg_to_m;
  m_buffer_lat = 2.0/m_lat_deg_to_m;
}
