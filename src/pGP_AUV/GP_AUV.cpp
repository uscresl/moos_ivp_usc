/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: GP.cpp                                               */
/*    DATE: 2015 - 2017                                          */
/*                                                               */
/*****************************************************************/

#include "GP_AUV.h"

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
#include "cg.h"

// write to file
#include <fstream>

// init srand
#include <chrono>

// include GraphNode
#include "GraphNode.h"
// reverse auto iterate
#include <boost/range/adaptor/reversed.hpp>

//---------------------------------------------------------
// Constructor
//
GP_AUV::GP_AUV() :
  m_verbose(true),
  m_input_var_data(""),
  m_input_var_sample_points(""),
  m_input_var_sample_points_specs(""),
  m_input_var_adaptive_trigger(""),
  m_output_var_pred(""),
  m_output_filename_prefix(""),
  m_prediction_interval(-1),
  m_path_planning_method("greedy"),
  m_debug(false),
  m_veh_name(""),
  m_use_log_gp(true),
  m_lat(0),
  m_lon(0),
  m_dep(0),
  m_surf_cnt(0),
  m_on_surface(false),
  m_adaptive(false),
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
  m_y_resolution(0.0),
  m_lon_deg_to_m(0.0),
  m_lat_deg_to_m(0.0),
  m_start_time(0.0),
  m_finding_nxt(false),
  m_mission_state(STATE_IDLE),
  m_gp( new libgp::GaussianProcess(2, "CovSum(CovSEiso, CovNoise)") ),
  m_hp_optim_running(false),
  m_final_hp_optim(false),
  m_hp_optim_iterations(50),
  m_hp_optim_cg(false),
  m_last_hp_optim_done(0),
  m_data_mail_counter(1),
  m_finished(false),
  m_downsample_factor(4),
  m_first_surface(true),
  m_area_buffer(5.0),
  m_bhv_state(""),
  m_recursive_greedy_budget(5),
  m_total_path_selection_time(0),
  m_total_paths_selected(0)
{
  // class variable instantiations can go here
  // as much as possible as function level initialization

  // usage: {lat,lon}_deg_to_m * degrees = m
  //        degrees = meters / {lat,long}_deg_to_m
  // TODO move this to library?
  m_lat_deg_to_m = 110923.99118801417;
  m_lon_deg_to_m = 92287.20804979937;

  // above we initialize a GP for 2D input data, //TODO convert to 3D
  // using the squared exponential covariance function,
  // with additive white noise

  // Set (log of) hyperparameters of the covariance function
  Eigen::VectorXd params(m_gp->covf().get_param_dim());
  // hyperparameters: length scale l^2, signal variance s_f^2, noise variance s_n^2
  // note, these will be optimized using cg or rprop
  // length scale: avg lat/lon_deg_to_m is 100000, 10m range = 0.0001
  //               50m = 0.0005, ln(0.0005) = -7.6
  // signal: from 0 to ca 30/40, log scale <0 to 1.5/1.6
  //         stdev 1.5, ln() = 0.4055
  // noise: let's set 10x smaller than signal
  //        stdev 0.15, ln() = -1.8971
  params << -7.6, 0.5, -1.8971;
  m_gp->covf().set_loghyper(params);

  // use a unique seed to initialize srand,
  // using milliseconds because vehicles can start within same second
  struct timeval time;
  gettimeofday(&time,NULL);
  int rand_seed = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  std::cout << GetAppName() << " :: rand_seed: " << rand_seed << std::endl;
  srand(rand_seed);

  std::map<int, GraphNode> sampling_locations;
}

GP_AUV::~GP_AUV()
{
  if ( m_ofstream_pm_lGP.is_open() )
    m_ofstream_pm_lGP.close();
  if ( m_ofstream_pv_lGP.is_open() )
    m_ofstream_pv_lGP.close();
  if ( m_ofstream_pmu_GP.is_open() )
    m_ofstream_pmu_GP.close();
  if ( m_ofstream_psigma2_GP.is_open() )
    m_ofstream_psigma2_GP.close();

  delete m_gp;
}

//---------------------------------------------------------
// Procedure: OnNewMail
//
// when variables are updated in the MOOSDB,
// there is 'new mail', check to see if
// there is anything for this process.
//
bool GP_AUV::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++)
  {
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
      // note, only add if we are in sampling state
      // (we do not want to add when in hp optim, or on surface etc)
      m_data_mail_counter++;

      if ( (m_data_mail_counter % 2 == 0) &&
            m_mission_state == STATE_SAMPLE )
        handleMailData(dval);
    }
    else if ( key == "NAV_LAT" )
      m_lat = dval;
    else if ( key == "NAV_LONG" )
      m_lon = dval;
    else if ( key == "NAV_DEPTH" )
    {
      m_dep = dval;

      if ( std::abs(m_dep) < 0.2 )
        m_surf_cnt++;
      else
        m_surf_cnt = 0;

      if ( m_surf_cnt > 10 )
        m_on_surface = true;
    }
    else if ( key == m_input_var_sample_points )
    {
      // store list of sample locations coming pSamplePoints
      // (done once)
      handleMailSamplePoints(sval);
    }
    else if ( key == m_input_var_sample_points_specs )
    {
      // store specs for sample points
      // (done once)
      handleMailSamplePointsSpecs(sval);
    }
    else if ( key == m_input_var_adaptive_trigger )
    {
      // check when adaptive waypoint reached
      // these are flags set by the adaptive wpt behavior
      // if we get it, it must mean a wpt/cycle was done
      if ( std::abs(m_last_published - MOOSTime()) > 1.0 &&
           (m_mission_state == STATE_SAMPLE || m_mission_state == STATE_IDLE ) )
      {
        std::cout << GetAppName() << " :: STATE_CALCWPT via m_input_var_adaptive_trigger" << std::endl;
        m_mission_state = STATE_CALCWPT;
        publishStates("OnNewMail_wpt_trigger");
      }
    }
    else if ( key == "LOITER_DIST_TO_POLY" )
      m_loiter_dist_to_poly = dval;
    else if ( key == "STAGE" )
      m_bhv_state = sval;
    else if ( key == "MISSION_TIME" )
    {
      std::cout << GetAppName() << " :: MISSION_TIME: " << sval << std::endl;
      if ( sval == "end" && !m_final_hp_optim )
      {
        // end of adaptive mission, switch to final hp optimization
        // (if not in it already)
        m_final_hp_optim = true;

        if ( m_bhv_state != "hpoptim" )
          m_Comms.Notify("STAGE","hpoptim");

        if ( m_mission_state == STATE_SAMPLE || m_mission_state == STATE_CALCWPT )
        {
          m_mission_state = STATE_SURFACING;
          publishStates("OnNewMail_MISSION_TIME");
        }

      }
    }
    else if ( key == "DB_UPTIME" )
    {
      m_db_uptime = dval;
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;

  }

  return(true);
}


//---------------------------------------------------------
// Procedure: OnConnectToServer
//
bool GP_AUV::OnConnectToServer()
{
//  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second
//
bool GP_AUV::Iterate()
{
  if ( m_lon == 0 && m_lat == 0 && m_dep == 0 )
    return true;
  else
  {
    if ( m_start_time < 1.0 )
      m_start_time = MOOSTime(); // first time, set the start time of process

    // **** SAVING GP TO FILE (independent) **********************************//
    if ( m_mission_state != STATE_DONE && !(m_hp_optim_running && m_final_hp_optim) )
    { // TODO consider if we want to run this during HP optimization stage (maybe all but last?)
      // periodically (every 600s = 10min), store all GP predictions
      // if we did not do so in the last second (apptick)
      if ( (std::abs(m_last_pred_save - MOOSTime()) > 1.0 ) &&
           ((size_t)std::floor(MOOSTime()-m_start_time) % 600 == 10) )
      {
        std::cout << GetAppName() << " :: saving state at mission time: " << std::floor(MOOSTime()-m_start_time) << std::endl;
        std::thread pred_store(&GP_AUV::makeAndStorePredictions, this);
        pred_store.detach();
        m_last_pred_save = MOOSTime();
      }
    }

    // **** HPOPTIM FOR 1 AUV **********************************************//
    // run hpoptim every .. 500 seconds ..
    if ( (std::abs(m_last_hp_optim_done - MOOSTime()) > 1.0 ) &&
         ((size_t)std::floor(MOOSTime()-m_start_time) % 500 == 10) )
    {
      if ( m_mission_state != STATE_DONE )
      {
        m_mission_state = STATE_HPOPTIM;
        publishStates("Iterate_hpoptim_1AUV");
      }
    }

    // **** MAIN STATE MACHINE *********************************************//
    if ( m_debug )
      std::cout << GetAppName() << " :: Current state: " << m_mission_state << std::endl;
    switch ( m_mission_state )
    {
      case STATE_SAMPLE :
        // just sampling, don't do anything else
        // TODO add in timed? trigger hp optim for parallel sampling?

        break;
      case STATE_CALCWPT :
        findAndPublishNextWpt();
        break;
      case STATE_SURFACING :
        if ( m_bhv_state != "data_sharing" && !m_final_hp_optim )
          Notify("STAGE","data_sharing");

        break;
      case STATE_HPOPTIM :
        startAndCheckHPOptim();
        break;
      case STATE_IDLE :
        break;
      default :
        break;
    }

  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open
bool GP_AUV::OnStartUp()
{
  CMOOSApp::OnStartUp();
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(true);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    std::cout << GetAppName() << " :: No config block found for " << GetAppName();

  initGeodesy();

  STRING_LIST::iterator p;
  for (p=sParams.begin(); p!=sParams.end(); p++)
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = true;
    if ( param == "verbose" )
      m_verbose = (value == "true") ? true : false;
    else if ( param == "input_var_data" )
      m_input_var_data = toupper(value);
    else if ( param == "input_var_sample_points" )
      m_input_var_sample_points = toupper(value);
    else if ( param == "input_var_sample_points_specs")
      m_input_var_sample_points_specs = toupper(value);
    else if ( param == "input_var_adaptive_trigger" )
      m_input_var_adaptive_trigger = toupper(value);
    else if ( param == "output_var_predictions" )
      m_output_var_pred = toupper(value);
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

      // prepare file actions
      std::string file_pm = (m_output_filename_prefix + "_pred_mean.csv");
      std::string file_pv = (m_output_filename_prefix + "_pred_var.csv");
      m_ofstream_pm_lGP.open(file_pm, std::ios::app);
      m_ofstream_pv_lGP.open(file_pv, std::ios::app);
      if ( m_use_log_gp )
      {
        std::string file_pmu = (m_output_filename_prefix + "_post_mu.csv");
        std::string file_psigma = (m_output_filename_prefix + "_post_sigma2.csv");
        m_ofstream_pmu_GP.open(file_pmu, std::ios::app);
        m_ofstream_psigma2_GP.open(file_psigma, std::ios::app);
      }
    }
    else if ( param == "use_log_gp" )
    {
      m_use_log_gp = (value == "true" ? true : false);
      if ( m_verbose )
        std::cout << GetAppName() << " :: using log GP? " << (m_use_log_gp ? "yes" : "no") << std::endl;
    }
    else if ( param == "downsample_factor" )
      m_downsample_factor = (size_t)atoi(value.c_str());
    else if ( param == "area_buffer" )
      m_area_buffer = (double)atof(value.c_str());
    else if ( param == "hp_optim_iterations" )
      m_hp_optim_iterations = (size_t)atoi(value.c_str());
    else if ( param == "adaptive" )
      m_adaptive = (value == "true" ? true : false);
    else if ( param == "hp_optim_method" )
    {
      // default: rprop
      m_hp_optim_cg = ( value == "cg" ) ? true : false;
    }
    else if ( param == "path_planning_method" )
    {
      if ( value == "greedy" || value == "dynamic_programming" || value == "recursive_greedy")
      {
        m_path_planning_method = value;
        if(m_verbose){
          std::cout << GetAppName() << " :: Path planning method: " << value << std::endl;
        }
      }
      else
      {
        std::cout << GetAppName() << " :: Error, unknown method. Choose from: greedy, "
                  << "dynamic_programming, recursive_greedy. Default: greedy." << std::endl;
        handled = false;
      }
    }
    else if ( param == "recursive_greedy_budget" )
    {
      m_recursive_greedy_budget = (size_t)atoi(value.c_str());
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

  // store vehicle name
  if ( !m_MissionReader.GetValue("Community",m_veh_name) )
  {
     m_veh_name = "error";
     std::cout << GetAppName() << " :: Unable to retrieve vehicle name! What is 'Community' set to?" << std::endl;
  }

  registerVariables();

  std::thread data_thread(&GP_AUV::dataAddingThread, this);
  data_thread.detach();

  // init last calculations times to start of process
  m_last_published = MOOSTime();

  return(true);
}

//---------------------------------------------------------
// Procedure: initGeodesy
//            initialize MOOS Geodesy for lat/lon conversions
//
void GP_AUV::initGeodesy()
{
  // get lat/lon origin from MOOS file
  bool failed = false;
  double latOrigin, longOrigin;
  if ( !m_MissionReader.GetValue("LatOrigin", latOrigin) )
  {
    std::cout << GetAppName() << " :: LatOrigin not set in *.moos file." << std::endl;
    failed = true;
  }
  else if ( !m_MissionReader.GetValue("LongOrigin", longOrigin) )
  {
    std::cout << GetAppName() << " :: LongOrigin not set in *.moos file" << std::endl;
    failed = true;
  }
  else {
    // initialize m_geodesy
    if ( !m_geodesy.Initialise(latOrigin, longOrigin) )
    {
      std::cout << GetAppName() << " :: Geodesy init failed." << std::endl;
      failed = true;
    }
  }
  if ( failed )
  {
    std::cout << GetAppName() << " :: failed to initialize geodesy, exiting." << std::endl;
    RequestQuit();
  }
  else
    std::cout << GetAppName() << " :: Geodesy initialized: lonOrigin, latOrigin = " << longOrigin << ", " << latOrigin << std::endl;
}


//---------------------------------------------------------
// Procedure: registerVariables
//            at startup, let the MOOSDB know what you want
//            to receive
//
void GP_AUV::registerVariables()
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

  // get current bhv state
  m_Comms.Register("STAGE", 0);

  // get trigger end mission
  m_Comms.Register("MISSION_TIME", 0);

  // get when wpt cycle finished
  // (when to run adaptive predictions in adaptive state)
  m_Comms.Register(m_input_var_adaptive_trigger, 0);

  // data sharing
  m_Comms.Register("LOITER_DIST_TO_POLY", 0);

  // db uptime for debugging
  m_Comms.Register("DB_UPTIME", 0);

  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: Done registering, registered for: ";
    std::set<std::string> get_registered = m_Comms.GetRegistered();
    for ( auto registered : get_registered )
      std::cout << registered << ", ";
    std::cout << std::endl;
  }
}

//---------------------------------------------------------
// Procedure: handleMailData
//            handle the incoming message
//
void GP_AUV::handleMailData(double received_data)
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

    // pass into data adding queue
    std::vector<double> nw_data_pt{veh_lon, veh_lat, received_data};
    m_queue_data_points_for_gp.push(nw_data_pt);
  }
}

//---------------------------------------------------------
// Procedure: handleMailSamplePoints
//            parse the string, store the sample locations
//
void GP_AUV::handleMailSamplePoints(std::string input_string)
{
  // input: semicolon separated string of comma separated locations
  // separate by semicolon
  std::vector<std::string> sample_points = parseString(input_string, ';');
  // for each, add to vector
  m_sample_graph_nodes.reserve(sample_points.size());

  std::ofstream ofstream_loc;
  std::string m_file_loc = (m_output_filename_prefix + "_locations.csv");
  ofstream_loc.open(m_file_loc, std::ios::out);

  m_lanes_x = std::floor(m_pts_grid_width / m_pts_grid_spacing);
  m_lanes_y = std::floor(m_pts_grid_height / m_pts_grid_spacing);
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

    // copy the points to have all sample points for easier processing for predictions
    m_sample_graph_nodes.emplace_back(loc_vec, 0.0);

    GraphNode* graph_node = &m_sample_graph_nodes.back();
    long left_neighbour_index = id_pt - (m_lanes_y + 1);
    long back_neighbour_index = id_pt - 1;
    GraphNode* left_neighbour = left_neighbour_index < 0 ? NULL : &m_sample_graph_nodes[left_neighbour_index];
    GraphNode* back_neighbour = back_neighbour_index < 0 || id_pt % (m_lanes_y + 1) == 0? NULL : &m_sample_graph_nodes[back_neighbour_index];
    graph_node->set_left_neighbour(left_neighbour);
    if ( left_neighbour )
    {
      left_neighbour->set_right_neighbour(graph_node);
    }
    graph_node->set_back_neighbour(back_neighbour);
    if ( back_neighbour )
    {
      back_neighbour->set_front_neighbour(graph_node);
    }

    // Alternative would be to have objects in maps or allocating them dynamically.
    m_sample_points_unvisited.insert( std::pair<size_t, GraphNode*>(id_pt, graph_node) );

    // save to file
    ofstream_loc << std::setprecision(15) << lon << ", " << lat << '\n';

    // the first location should be bottom left corner, store as minima
    if ( id_pt == 0 )
    {
      m_min_lon = lon;
      m_min_lat = lat;
      if ( m_debug )
        std::cout << GetAppName() << " :: SW Sample location: " << std::setprecision(10) << lon << " " << lat << std::endl;
    }
    if ( id_pt == sample_points.size()-1 )
    {
      // last location = top right corner, store as maxima
      m_max_lon = lon;
      m_max_lat = lat;
      if ( m_debug )
        std::cout << GetAppName() << " :: NE Sample location: " << std::setprecision(10) << lon << " " << lat << std::endl;
    }
  }
  ofstream_loc.close();
  // check / communicate what we did
  if ( m_debug )
    std::cout << GetAppName() << " :: stored " << m_sample_points_unvisited.size() << " sample locations" << std::endl;
}

//---------------------------------------------------------
// Procedure: storeSamplePointsSpecs
//            parse the string, store specs for sample locations
//
void GP_AUV::handleMailSamplePointsSpecs(std::string input_string)
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
      calcLonLatSpacing();
    }
    else
      std::cout << GetAppName() << " :: error, unhandled part of sample points specs: " << param << std::endl;
  }
  if ( m_verbose )
    std::cout << GetAppName() << " :: width, height, spacing: " << m_pts_grid_width << ", " <<
                 m_pts_grid_height << ", " << m_pts_grid_spacing << std::endl;
}

//---------------------------------------------------------
// Procedure: dataAddingThread()
//            continuously check the data point queue to see
//            if points need to be added to the GP, if so
//            then call func to add points to GP
//
void GP_AUV::dataAddingThread()
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

      if ( m_debug )
        std::cout << GetAppName() << " :: adding data point: " << veh_lon
                  << "," << veh_lat << ":" << data_pt[2] << std::endl;

      // update visited set if needed
      int index = getIndexForMap(veh_lon, veh_lat);
      if ( index >= 0 && needToUpdateMaps((size_t)index) )
        updateVisitedSet(veh_lon, veh_lat, (size_t)index);
    }
  }
}

//---------------------------------------------------------
// Procedure: addPatternToGP(double veh_lon, double veh_lat, double value)
//            function to add points to GP, using mutex so as
//            to not disturb when other processes are reading from GP
//
void GP_AUV::addPatternToGP(double veh_lon, double veh_lat, double value)
{
  // limit scope mutex, protect when adding data
  // because this is now happening in a detached thread
  double location[2] = {veh_lon, veh_lat};

  // GP: pass in value
  // log GP: take log (ln) of measurement
  double save_val = m_use_log_gp ? log(value) : value;

  // downsampled data for hyperparam optimization
  if ( m_gp->get_sampleset_size() % m_downsample_factor == 0 )
  {
    std::vector<double> nw_data_pt{veh_lon, veh_lat, save_val};
    m_data_for_hp_optim.push(nw_data_pt);
  }

  std::unique_lock<std::mutex> ap_lock(m_gp_mutex, std::defer_lock);
  // obtain lock
  while ( !ap_lock.try_lock() ) {}
  // Input vectors x must be provided as double[] and targets y as double.
  // add new data point to GP
  m_gp->add_pattern(location, save_val);
  // release mutex
  ap_lock.unlock();
}


//---------------------------------------------------------
// Procedure: needToUpdateMaps
//            check if location's grid index is in unvisited map
//
bool GP_AUV::needToUpdateMaps(size_t grid_index)
{
    // add mutex for changing of global maps
    std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
    while ( !map_lock.try_lock() ){}
    std::unordered_map<size_t, GraphNode*>::iterator curr_loc_itr = m_sample_points_unvisited.find(grid_index);
    map_lock.unlock();

    return ( curr_loc_itr != m_sample_points_unvisited.end() );
}

//---------------------------------------------------------
// Procedure: getIndexForMap
//            calculate the grid index for the vehicle location
//
int GP_AUV::getIndexForMap(double veh_lon, double veh_lat)
{
  if ( inSampleRectangle(veh_lon, veh_lat, true) )
  {
    // calculate the id of the location where the vehicle is currently
    // at, and move it to visited
    size_t x_cell_rnd = (size_t) round((veh_lon - m_min_lon)/m_lon_spacing);
    size_t y_cell_rnd = (size_t) round((veh_lat - m_min_lat)/m_lat_spacing);

    // calculate index into map (stored from SW, y first, then x)
    int index = m_y_resolution*x_cell_rnd + y_cell_rnd;

    return index;
  }
  else
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: Location not in sample rectangle." << std::endl;
  }
  return -1;
}

//---------------------------------------------------------
// Procedure: updateVisitedSet
//            given current vehicle location, move locations
//            from unvisited to visited set
//
void GP_AUV::updateVisitedSet(double veh_lon, double veh_lat, size_t index )
{
  // add mutex for changing of global maps
  std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
  while ( !map_lock.try_lock() ){}

  std::unordered_map<size_t, GraphNode*>::iterator curr_loc_itr = m_sample_points_unvisited.find(index);
  if ( curr_loc_itr != m_sample_points_unvisited.end() )
  {
    // remove point from unvisited set
    Eigen::Vector2d move_pt = curr_loc_itr->second->get_location();

    // check if the sampled point was nearby, if not, there's something wrong
    bool pt_nearby = checkDistanceToSampledPoint(veh_lon, veh_lat, move_pt);
    if ( !pt_nearby && m_mission_state == STATE_SAMPLE )
    {
      std::cout << GetAppName() << " :: ERROR? distance to sampled point is bigger than the grid spacing\n";
    }

    // add the point to the visited set
    m_sample_points_visited.insert(std::pair<size_t, GraphNode*>(index, curr_loc_itr->second));

    // and remove it from the unvisited set
    m_sample_points_unvisited.erase(curr_loc_itr);

    // report
    if ( m_verbose )
    {
      std::cout << '\n' << GetAppName() << " :: moved pt: " << std::setprecision(10) << move_pt(0) << ", " << move_pt(1);
      std::cout << " from unvisited to visited.\n";
      std::cout << GetAppName() << " :: Unvisited size: " << m_sample_points_unvisited.size() << '\n';
      std::cout << GetAppName() << " :: Visited size: " << m_sample_points_visited.size() << '\n' << std::endl;
    }
  }

  map_lock.unlock();
}

//---------------------------------------------------------
// Procedure: checkDistanceToSampledPoint
//            we check if, after conversion, the distance
//            of sampled point to vehicle location is reasonable
//
bool GP_AUV::checkDistanceToSampledPoint(double veh_lon, double veh_lat, Eigen::Vector2d move_pt)
{
  double dist_lon = std::abs(move_pt(0) - veh_lon);
  double dist_lat = std::abs(move_pt(1) - veh_lat);
  double dist_lon_m = dist_lon*m_lon_deg_to_m;
  double dist_lat_m = dist_lat*m_lat_deg_to_m;

  if ( dist_lon_m > m_pts_grid_spacing || dist_lat_m > m_pts_grid_spacing )
  {
    std::cout << GetAppName() << " :: Distance to chosen point: " << dist_lon_m << " " << dist_lat_m << '\n';
    return false;
  }

  return true;
}

//---------------------------------------------------------
// Procedure: kickOffCalcMetric
//            we need to find the next sampling locations:
//            launch the maximum entropy calculations,
//            and store results to map
//
void GP_AUV::kickOffCalcMetric()
{
  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: kick off calculation metric (MECriterion), "
              << "or choose random sampling location if GP empty." << std::endl;
  }

  if ( checkGPHasData() )
  {
    // for each y (from unvisited set only, as in greedy algorithm Krause'08)
    // calculate the mutual information term
    if ( m_sample_points_unvisited.size() > 0 )
    {
      if ( !m_finding_nxt )
      {
        // use threading because this is a costly operation that would otherwise
        // interfere with the MOOS app rate
        m_future_next_pt = std::async(std::launch::async, &GP_AUV::calcMECriterion, this);
        m_finding_nxt = true;
      }
    }
    else
      std::cout << GetAppName() << " :: m_sample_points_unvisited is empty, unable to find next location" << std::endl;
  }
  else
  {
    std::cout << GetAppName() << " :: GP is empty. Getting random location for initial sample location." << std::endl;
    getRandomStartLocation();
  }
}

//---------------------------------------------------------
// Procedure: getRandomStartLocation
//            before there is data in the GP,
//            grab a random location to start from
//
void GP_AUV::getRandomStartLocation()
{
  int random_idx = (int)(rand() % (m_sample_graph_nodes.size()));
  Eigen::Vector2d rand_loc = m_sample_graph_nodes.at(random_idx).get_location();

  std::ostringstream output_stream;
  output_stream << std::setprecision(15) << rand_loc(0) << "," << rand_loc(1);
  m_Comms.Notify(m_output_var_pred, output_stream.str());

  // update state vars
  m_mission_state = STATE_SAMPLE;
  publishStates("getRandomStartLocation");
  m_last_published = MOOSTime();
}


//---------------------------------------------------------
// Procedure: greedyWptSelection()
//            check over all predictions to find best:
//            greedy choice to maximize entropy for unvisited
//            locations
//
void GP_AUV::greedyWptSelection(std::string & next_waypoint)
{
  Eigen::Vector2d best_location;

  // get next position, greedy pick (max entropy location)
  double best_so_far = -1*std::numeric_limits<double>::max();
  size_t best_so_far_idx = -1;

  // check for all unvisited locations
  for ( auto loc : m_sample_points_unvisited )
  {
    if ( loc.second->get_value() > best_so_far )
    {
      best_so_far = loc.second->get_value();
      best_so_far_idx = loc.first;
    }
  }

  if ( m_verbose )
    std::cout << GetAppName() << " ::  best so far: (idx, val) " << best_so_far_idx << ", " << best_so_far << std::endl;

  if ( best_so_far_idx >= 0 )
  {
    auto best_itr = m_sample_points_unvisited.find(best_so_far_idx);

    if ( best_itr == m_sample_points_unvisited.end() )
      std::cout << GetAppName() << " :: Error: best is not in unsampled locations" << std::endl;
    else
    {
      best_location = best_itr->second->get_location();

      // app feedback
      if ( m_verbose )
      {
        std::cout << GetAppName() << " :: publishing " << m_output_var_pred << '\n';
        std::cout << GetAppName() << " :: current best next y: " << std::setprecision(15) << best_location(0) << ", " << best_location(1) << '\n';
      }
    }
  }
  else
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: invalid index: " << best_so_far_idx << ", not publishing." << std::endl;
    best_location(0) = 0;
    best_location(1) = 0;
  }

  // make a string from the single lon/lat location
  std::ostringstream output_stream;
  output_stream << std::setprecision(15) << best_location(0) << "," << best_location(1);
  next_waypoint = output_stream.str();
}

//---------------------------------------------------------
// Procedure: getX
//            we compute the X axis index of a GraphNode
//            given it's index in m_sample_graph_nodes
//
size_t GP_AUV::getX(size_t id_pt)
{
  return id_pt / (m_lanes_y + 1);
}

/*
// Procedure: dynamicProgrammingWptSelection()
//            check over all predictions to find best (dynamic programming method)
//
void GP_AUV::dynamicProgrammingWptSelection(Eigen::Vector2d & best_location) {
  // needs:
  // 1. how to calculate upper and lower bounds
  // 2. how to determine user-defined value a

  // continue algorithm if difference between upper and lower bounds is greater than user-specified bounds
  double upper_bounds;
  double lower_bounds;
  double user_specified_bounds;
  std::vector<double> posterior_entropy_values;

  if (upper_bounds - lower_bounds > user_specified_bounds)
  {
    // create vector of posterior entropy values to use for dynamic programming
    //TODO use posterior entropy values from map, do not calculate again
  }

  // find best possible path and the neighbor that leads to it
  //calcMaxME(posterior_entropy_values, sampling_locations, ); // figure out index to pass in

  // backtrack and update upper and lower bounds
  upper_bounds = 0;// max entropy calculation
  lower_bounds = 0;// min entropy calculation
}
*/


// ---------------------------------------------------------
// Procedure: dynamicWptSelection()
//            Go through sample locations map of graph nodes
//            Calculate value of path for a certain length
//            Choose the path of the highest value
void GP_AUV::dynamicWptSelection(std::string & next_waypoint)
{
  // is this the point we want to start calculating from?
//  std::cout << GetAppName() << " :: " << "calling dynWptSelection" << std::endl;
  int current_node_index = getIndexForMap(m_lon, m_lat);
//  std::cout << GetAppName() << " :: " << "current index: " << current_node_index << std::endl;
  auto itr = m_sample_points_visited.find(current_node_index);
  if ( itr != m_sample_points_visited.end() )
  {
    std::vector<const GraphNode *> nextWaypoints(5);

    // figure out how to get graph node from unvisited map
    int steps = 0;
//    std::cout << GetAppName() << " :: " << "itr index: " << itr->first << std::endl;
    const GraphNode* val = maxPath(itr->second, nextWaypoints, steps);
    std::cout << GetAppName() << " :: " << (val->get_location())(0) << "," << (val->get_location())(1) << std::endl;

    // publish next waypoints
    // make a string from the single lon/lat locations
    std::ostringstream output_stream;
    for (int i = 0; i < nextWaypoints.size(); i++)
    {
        Eigen::Vector2d nodeLoc = nextWaypoints[i]->get_location();
        output_stream << std::setprecision(15) << nodeLoc(0) << "," << nodeLoc(1);
        if ( i < (nextWaypoints.size() - 1) )
          output_stream << ":";
    }
    //Eigen::Vector2d nodeLoc = nextWaypoints[0]->get_location();

    std::cout << GetAppName() << " :: next waypoints: " << output_stream.str() << std::endl;
    next_waypoint = output_stream.str();
  }
  else
  {
      std::cout << GetAppName() << " :: unvisited node not found" << std::endl;
  }

}

//---------------------------------------------------------
// Procedure: maxPath()
//            helper function to help find the largest sum path
//            publish waypoints as they are selected
const GraphNode* GP_AUV::maxPath(const GraphNode* loc, std::vector<const GraphNode *>& toPublish, int steps)
{
    // need some sort of base case
    // base case: if 5 steps in, return loc and construct the path backwards
//    std::cout << GetAppName() << " :: " << "entered maxpath" << std::endl;
    if ( loc == nullptr )
    {
        return nullptr;
    }
    else if ( steps == 5 )
    {
//        return loc->get_value();
        std::cout << GetAppName() << " :: " << "steps = 5" << std::endl;
        return loc;
    }

    else {
        steps++;
//        std::cout << GetAppName() << " :: " << "inc steps" << std::endl;
//        std::cout << GetAppName() << " :: " << "call max function" << std::endl;

        const GraphNode* next = max(maxPath(loc->get_left_neighbour(), toPublish, steps),
                             maxPath(loc->get_right_neighbour(), toPublish, steps),
                             maxPath(loc->get_front_neighbour(), toPublish, steps)
        );
        toPublish[steps - 1] = next;
//        return loc->get_value() + next->get_value();
        return next;
    }

}

/*
//---------------------------------------------------------
// Procedure: calcMaxME
//            dynamic programming method to calculate max entropy
//
int GP_AUV::calcMaxME(std::vector<double> post_entropy_values, std::map<int, GraphNode> sampling_locations, int index) {
  if (index == 0) {
    return post_entropy_values[index];
  }
  else {
    return -1;
    //TODO std::max can only take 2 arguments
//    return post_entropy_values[index]
//          + std::max(calcMaxME(post_entropy_values, sampling_locations, findIndexOfNode(sampling_locations[index].get_left_neighbour())),
//                calcMaxME(post_entropy_values, sampling_locations, findIndexOfNode(sampling_locations[index].get_right_neighbour())),
//                calcMaxME(post_entropy_values, sampling_locations, findIndexOfNode(sampling_locations[index].get_front_neighbour())) );

  }
}
*/
/*
//---------------------------------------------------------
// Procedure: findIndexOfNode()
//
//
int GP_AUV::findIndexOfNode(const GraphNode * node) {
  std::map<int, GraphNode>::iterator it;
//  for (it = sampling_locations.begin(); it != sampling_locations.end(); ++it) {
    //TODO implement operator== for graph nodes?
//    if (it->second == *node) {
//      return it->first;
//    }
//  }

}
*/

/*
// this function should be replaced by the new calculate maximum entropy function

//---------------------------------------------------------
// Procedure: calcME
//            for each unvisited neighbor
//            calculate the max ME for a path
//            starting at that neighbor
//            return the neighbor that would produce path with max ME
//
// void GP_AUV::calcME() {
//   if ( m_verbose )
//     std::cout << GetAppName() << " :: max entropy start" << std::endl;
//   m_unvisited_pred_metric.clear();
//
//   std::clock_t begin = std::clock();
//   if ( m_debug )
//     std::cout << GetAppName() << " :: try for lock gp" << std::endl;
//   // lock for access to m_gp
//   std::unique_lock<std::mutex> gp_lock(m_gp_mutex, std::defer_lock);
//   // use unique_lock here, such that we can release mutex after m_gp operation
//   while ( !gp_lock.try_lock() ) {}
//   if ( m_debug )
//     std::cout << GetAppName() << " :: make copy GP" << std::endl;
//   libgp::GaussianProcess * gp_copy = new libgp::GaussianProcess(*m_gp);
//   // release lock
//   gp_lock.unlock();
//
//   if ( m_debug )
//     std::cout << GetAppName() << " :: try for lock map" << std::endl;
//   std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
//   while ( !map_lock.try_lock() ){}
//   // make copy of map to use instead of map,
//   // such that we do not have to lock it for long
//   std::unordered_map<size_t, Eigen::Vector2d, std::hash<size_t>, std::equal_to<size_t>, Eigen::aligned_allocator<std::pair<size_t, Eigen::Vector2d> > > unvisited_map_copy;
//   // calculate for all, because we need it for density voronoi calc for other vehicles
//   unvisited_map_copy.insert(m_sample_points_unvisited.begin(), m_sample_points_unvisited.end());
//   map_lock.unlock();
//
//   if ( m_debug )
//     std::cout << GetAppName() << " :: calc max entropy, size map: " << unvisited_map_copy.size() << std::endl;
//
//   // create vector of posterior entropy values to use for dynamic programming
//   std::vector<double> posterior_entropy_values;
//
//   // for each unvisited location
//   for ( auto y_itr : unvisited_map_copy )
//   {
//     // get unvisited location
//     Eigen::Vector2d y = y_itr.second;
//     double y_loc[2] = {y(0), y(1)};
//
//     // calculate its posterior entropy
//     double pred_mean;
//     double pred_cov;
//     gp_copy->f_and_var(y_loc, pred_mean, pred_cov);
//
//     // normal distribution
//     //  1/2 ln ( 2*pi*e*sigma^2 )
//     double post_entropy;
//     if ( !m_use_log_gp )
//       post_entropy = log( 2 * M_PI * exp(1) * pred_cov);
//     else
//     {
//       // lognormal distribution
//       double var_part = (1/2.0) * log( 2 * M_PI * exp(1) * pred_cov);
//       post_entropy = var_part + pred_mean;
//     }
//
//     m_unvisited_pred_metric.insert(std::pair<size_t, double>(y_itr.first, post_entropy));
//     posterior_entropy_values.push_back(post_entropy);
//   }
//
//   std::clock_t end = std::clock();
//   if ( m_verbose )
//     std::cout << GetAppName() << " :: Max Entropy calc time: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;
//
//   // copy of GP and unvisited_map get destroyed when this function exits
//   delete gp_copy;
//   return 0;
// }
*/

//---------------------------------------------------------
// Procedure: getY
//            we compute the Y axis index of a GraphNode
//            given it's index in m_sample_graph_nodes
//
size_t GP_AUV::getY(size_t id_pt)
{
  return id_pt % (m_lanes_y + 1);
}

//---------------------------------------------------------
// Procedure: manhattanDistance
//            we compute the manhattan distance between
//            two graph nodes in m_sample_graph_nodes
//
double GP_AUV::manhattanDistance(size_t start_node_index, size_t end_node_index)
{
  size_t start_x = getX(start_node_index), start_y = getY(start_node_index);
  size_t end_x = getX(end_node_index), end_y = getY(end_node_index);
  size_t dx = std::max(start_x, end_x) - std::min(start_x, end_x);
  size_t dy = std::max(start_y, end_y) - std::min(start_y, end_y);
  return dx + dy;
}

//---------------------------------------------------------
// Procedure: informativeValue
//            we calculate a path's informative value by summing values of all nodes in it
//
double GP_AUV::informativeValue(std::vector< size_t > cur_path)
{
  double path_informative_value = 0.0;
  for ( size_t node_index : cur_path )
  {
    path_informative_value += m_sample_graph_nodes[node_index].get_value();
  }
  return path_informative_value;
}

//---------------------------------------------------------
// Procedure: generalizedRecursiveGreedy
//            determine most informative path between start and end nodes
//            using recursive greedy algorithm
//
std::vector< size_t > GP_AUV::generalizedRecursiveGreedy(size_t start_node_index, size_t end_node_index, std::set< size_t > ground_set, size_t budget)
{
  std::vector< size_t > best_path;
  double manhattan_distance = manhattanDistance(start_node_index, end_node_index);
  // check if manhattan distance between start and end node is within budget
  if ( manhattan_distance <= budget )
  {
    if ( budget == 1 )
    {
      // when budget is one end node can only be start node's neighbour
      if ( ground_set.find(end_node_index) == ground_set.end() )
      {
        GraphNode* end_node = &m_sample_graph_nodes[end_node_index];
        std::vector< GraphNode* > start_node_neighbours = m_sample_graph_nodes[start_node_index].get_neighbours();
        for ( GraphNode* start_node_neighbour : start_node_neighbours )
        {
          if ( start_node_neighbour == end_node )
          {
            // if end node is a neighbour of start node, add them to the path
            best_path.push_back(start_node_index);
            best_path.push_back(end_node_index);
            break;
          }
        }
      }
    }
    else
    {
      double best_informative_value = -1 * std::numeric_limits< double >::max();
      // check all nodes as middle node between start and end node to find the best path
      for ( size_t middle_node_index = 0; middle_node_index < m_sample_graph_nodes.size(); middle_node_index++ )
      {
        if ( middle_node_index != start_node_index && middle_node_index != end_node_index && ground_set.find(middle_node_index) == ground_set.end() )
        {
          // check all budgets possible to find the best path
          for ( size_t middle_budget = manhattanDistance(start_node_index, middle_node_index);
                middle_budget <= budget - manhattanDistance(middle_node_index, end_node_index); middle_budget++ )
          {
            std::set< size_t > first_ground_set(ground_set.begin(), ground_set.end());
            first_ground_set.insert(end_node_index);
            std::vector< size_t > first_half_path = generalizedRecursiveGreedy(start_node_index, middle_node_index, first_ground_set, middle_budget);
            // if a valid path exists from start to middle node find path from middle to end node.
            if ( !first_half_path.empty() )
            {
              std::set< size_t > new_ground_set(ground_set.begin(), ground_set.end());
              new_ground_set.insert(first_half_path.begin(), first_half_path.end());
              std::vector< size_t > second_half_path = generalizedRecursiveGreedy(middle_node_index, end_node_index, new_ground_set,
                budget - first_half_path.size() + 1);
              if ( !second_half_path.empty() )
              {
                // concatenate two paths to get path from start to end node
                std::vector< size_t > & cur_path = first_half_path;
                cur_path.insert(cur_path.end(), second_half_path.begin() + 1, second_half_path.end());
                double cur_path_informative_value = informativeValue(cur_path);
                // update best path if current path is more informative
                if ( cur_path_informative_value > best_informative_value )
                {
                  best_path = cur_path;
                  best_informative_value = cur_path_informative_value;
                }
              }
            }
          }
        }
      }
    }
  }
  return best_path;
}

//---------------------------------------------------------
// Procedure: recursiveGreedyWptSelection()
//            check over all predictions from generalized
//            recursive greedy algorithm to find best next way point
//
void GP_AUV::recursiveGreedyWptSelection(std::string & next_waypoint)
{
  std::clock_t begin = std::clock();
  if ( m_debug )
    std::cout << GetAppName() << " :: Recursive Greedy Waypoint Selection" << std::endl;
  // get next position, greedy pick from the paths returned by GRG algorithm
  double best_so_far = -1 * std::numeric_limits< double >::max();
  std::vector< size_t > best_path_so_far;
  // Get current vehicle location's index on map
  int current_node_index = getIndexForMap(m_lon, m_lat);
  if ( current_node_index < 0 )
  {
    std::cout << GetAppName() << " :: Error: vehicle location is not in sample rectangle" << std::endl;
  }
  else if ( current_node_index >= m_sample_graph_nodes.size() )
  {
    std::cout << GetAppName() << " :: Error: vehicle index is not in sample graph nodes" << std::endl;
  }
  else
  {
    // calculate path from current node to all nodes within budget
    for ( size_t i = 0; i < m_sample_graph_nodes.size(); i++ )
    {
      if ( manhattanDistance(current_node_index, i) >= m_recursive_greedy_budget )
      {
        std::set< size_t > ground_set;
        std::vector< size_t > cur_path = generalizedRecursiveGreedy(current_node_index, i, ground_set, m_recursive_greedy_budget);
        if ( !cur_path.empty() )
        {
          double cur_path_value = informativeValue(cur_path);
          // change best path if current calculated path is more informative
          if ( m_debug )
            std::cout << GetAppName() << " :: Curr Value: " << cur_path_value << std::endl;
          if ( cur_path_value > best_so_far )
          {
            best_so_far = cur_path_value;
            best_path_so_far = cur_path;
          }
        }
      }
    }

    if ( m_verbose )
    {
      std::cout << GetAppName() << " ::  best path so far: (val, [ path ]) " << best_so_far << ", [ ";
      for ( int i = 0; i < best_path_so_far.size(); i++ )
      {
        if ( i != 0 )
          std::cout << ", ";
        std::cout << best_path_so_far[i];
      }
      std::cout << " ]" << std::endl;
    }

    // create string of nodes in the best path
    std::ostringstream output_stream;
    for ( size_t i = 0; i < best_path_so_far.size(); i++ )
    {
      Eigen::Vector2d node_loc;
      size_t node_idx = best_path_so_far[i];
      if ( node_idx >= 0 )
      {
        if ( node_idx >= m_sample_graph_nodes.size() )
        {
          std::cout << GetAppName() << " :: Error: path node is not a possible sample locations" << std::endl;
          break;
        }
        else
        {
          node_loc = m_sample_graph_nodes[node_idx].get_location();

          // app feedback
          if ( m_verbose )
          {
            std::cout << GetAppName() << " :: publishing " << m_output_var_pred << '\n';
            std::cout << GetAppName() << " :: current path location y: " << std::setprecision(15) << node_loc(0) << ", " << node_loc(1) << '\n';
          }
        }
      }
      else
      {
        if ( m_debug )
          std::cout << GetAppName() << " :: invalid index: " << node_idx << ", not publishing." << std::endl;
        node_loc(0) = 0;
        node_loc(1) = 0;
        break;
      }
      if ( i ) output_stream << ":";
      output_stream << std::setprecision(15) << node_loc(0) << "," << node_loc(1);
    }
    next_waypoint = output_stream.str();
  }
  std::clock_t end = std::clock();
  double path_selection_time = double(end-begin)/CLOCKS_PER_SEC;
  m_total_paths_selected++;
  m_total_path_selection_time += path_selection_time;
  if ( m_debug )
  {
    std::cout << GetAppName() << ":: Path selected in " << path_selection_time << std::endl;
    std::cout << GetAppName() << ":: Total paths selected: " << m_total_paths_selected << std::endl;
    std::cout << GetAppName() << ":: Total path selection time: " << m_total_path_selection_time << std::endl;
    std::cout << GetAppName() << ":: Avg path selection time: " << m_total_path_selection_time/m_total_paths_selected << std::endl;
  }
}

//---------------------------------------------------------
// Procedure: publishNextWaypointLocations
//            call MOOS's Notify & publish location(s)
//
void GP_AUV::publishNextWaypointLocations()
{
  // procedures for finding the next waypoint(s)
  std::cout << GetAppName() << " :: Path planning method: " << m_path_planning_method << std::endl;
  std::string next_wpts("");
  if ( m_path_planning_method == "greedy" )
  {
    greedyWptSelection(next_wpts);
  }
  else if ( m_path_planning_method == "dynamic_programming" )
  {
    dynamicWptSelection(next_wpts);
  }
  else if ( m_path_planning_method == "recursive_greedy" )
  {
    recursiveGreedyWptSelection(next_wpts);
  }

  if ( next_wpts == "" )
    std::cout << GetAppName() << " :: Error: no waypoint found yet." << std::endl;
  else
  {
    // publishing for behavior (pLonLatToWptUpdate)

    m_Comms.Notify(m_output_var_pred, next_wpts);

    // update state vars
    m_last_published = MOOSTime();
    m_mission_state = STATE_SAMPLE;
    publishStates("publishNextBestPosition");
  }
}

//---------------------------------------------------------
// Procedure: calcMECriterion
//            calculate maximum entropy
//            for every unvisited location,
//            and pick best (greedy)
//
size_t GP_AUV::calcMECriterion()
{
  if ( m_verbose )
    std::cout << GetAppName() << " :: max entropy start" << std::endl;

  std::clock_t begin = std::clock();
  if ( m_debug )
    std::cout << GetAppName() << " :: try for lock gp" << std::endl;
  // lock for access to m_gp
  std::unique_lock<std::mutex> gp_lock(m_gp_mutex, std::defer_lock);
  // use unique_lock here, such that we can release mutex after m_gp operation
  while ( !gp_lock.try_lock() ) {}
  if ( m_debug )
    std::cout << GetAppName() << " :: make copy GP" << std::endl;
  libgp::GaussianProcess * gp_copy = new libgp::GaussianProcess(*m_gp);
  // release lock
  gp_lock.unlock();

  if ( m_debug )
    std::cout << GetAppName() << " :: try for lock map" << std::endl;
  std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
  while ( !map_lock.try_lock() ){}
  // make copy of map to use instead of map,
  // such that we do not have to lock it for long
  std::vector<GraphNode> sample_graph_node_copy(m_sample_graph_nodes.begin(), m_sample_graph_nodes.end());
  // calculate for all, because we need it for density voronoi calc for other vehicles
  map_lock.unlock();

  if ( m_debug )
    std::cout << GetAppName() << " :: calc max entropy, size map: " << sample_graph_node_copy.size() << std::endl;

  // for each unvisited location
  std::vector<double> new_values;
  for ( auto y_itr : sample_graph_node_copy )
  {
    // get unvisited location
    Eigen::Vector2d yy = y_itr.get_location();
    if ( m_debug )
      std::cout << GetAppName() << " :: calculating entropy for: " << yy(0) << ", " << yy(1) << std::endl;
    double y_loc[2] = {yy(0), yy(1)};

    // calculate its posterior entropy
    double pred_mean;
    double pred_cov;
    gp_copy->f_and_var(y_loc, pred_mean, pred_cov);

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

    if ( m_debug )
      std::cout << GetAppName() << " :: calculated entropy: " << post_entropy << std::endl;
    new_values.push_back(post_entropy);
    if ( m_debug )
      std::cout << GetAppName() << " :: new value: " << new_values.back() << std::endl;
  }

  while ( !map_lock.try_lock() ){}
  // update map entropies from calculated values in copy
  for ( size_t idx = m_sample_graph_nodes.size(); idx > 0 ; --idx )
  {
    GraphNode* item = &m_sample_graph_nodes[idx];
    double val = new_values.back();
    if ( m_verbose )
      std::cout << GetAppName() << " :: setting value to: " << val << std::endl;
    item->set_value(val);
    new_values.pop_back();
  }
  map_lock.unlock();

  std::clock_t end = std::clock();
  if ( m_verbose )
    std::cout << GetAppName() << " :: Max Entropy calc time: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  // copy of GP and unvisited_map get destroyed when this function exits
  delete gp_copy;
  return 0;
}


//---------------------------------------------------------
// Procedure: checkGPHasData
//            check that the GP has been filled, else quit
//
bool GP_AUV::checkGPHasData()
{
  return ( m_gp->get_sampleset_size() > 0 );
}


//---------------------------------------------------------
// Procedure: startAndCheckHPOptim()
//            start HP optim if not running yet (thread)
//            if running, check if done
//            if done, (a) continue sampling, or
//                     (b) end the mission
//
void GP_AUV::startAndCheckHPOptim()
{
  // start HP optimization, if not running already
  if ( !m_hp_optim_running )
  {
    // start thread for hyperparameter optimization,
    // because this will take a while..
    // TODO parameterize nr iterations
    m_future_hp_optim = std::async(std::launch::async, &GP_AUV::runHPOptimization, this, 100);
    if ( m_verbose )
      std::cout << GetAppName() << " :: Starting hyperparameter optimization, current size GP: " << m_gp->get_sampleset_size() << std::endl;
    m_hp_optim_running = true;
  }
  else
  {
    // if running, check if done
    if ( m_future_hp_optim.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
    {
        bool hp_optim_done = m_future_hp_optim.get(); // should be true
        if ( !hp_optim_done )
        {
          std::cout << GetAppName() << " :: ERROR: should be done with HP optimization, but get() returns false!" << std::endl;
          return;
        }
        else
        {
          m_last_hp_optim_done = (size_t)std::floor(MOOSTime());
          if ( m_verbose )
            std::cout << GetAppName() << " :: Done with hyperparameter optimization. New HPs: " << m_gp->covf().get_loghyper() << std::endl;

          std::cout << GetAppName() << " :: tdsResetStateVars via hpoptimdone" << std::endl;
          tdsResetStateVars();

          // after final HP optim
          if ( m_final_hp_optim )
            endMission();

          m_hp_optim_running = false;
        }
      }
      else
        std::cout << GetAppName() << " :: waiting for hp optim" << std::endl;
  }

}

//---------------------------------------------------------
// Procedure: endMission()
//            set variables to end mission, initiate last save
//
void GP_AUV::endMission()
{
  m_mission_state = STATE_DONE;
  publishStates("endMission");

  m_Comms.Notify("STAGE","return");

  // store predictions after HP optim
  m_finished = true;
  std::thread pred_store(&GP_AUV::makeAndStorePredictions, this);
  pred_store.detach();
}


//---------------------------------------------------------
// Procedure: runHPOptimization(nr_iterations)
//            run in thread, call GP's hyperparam optimization
//
bool GP_AUV::runHPOptimization(size_t nr_iterations)
{

  // run hyperparameter optimization

  // protect GP access with mutex
  if ( m_verbose)
  {
    std::cout << GetAppName() << " :: continuing HP optimization" << std::endl; //obtained lock,
    std::cout << GetAppName() << " :: current size GP: " << m_gp->get_sampleset_size() << std::endl;
  }

  Eigen::VectorXd lh_gp(m_gp->covf().get_loghyper()); // via param function
  runHPoptimizationOnDownsampledGP(lh_gp, nr_iterations);

  // pass on params to GP
  Eigen::VectorXd hparams(lh_gp);

  std::unique_lock<std::mutex> hp_lock(m_gp_mutex, std::defer_lock);
  while ( !hp_lock.try_lock() ){}
  m_gp->covf().set_loghyper(hparams);
  // just update hyperparams. Call for f and var should init re-compute.
  hp_lock.unlock();

  if ( m_verbose )
    std::cout << GetAppName() << " :: new m_GP hyper params: " << m_gp->covf().get_loghyper() << std::endl;

  return true;
}

void GP_AUV::runHPoptimizationOnDownsampledGP(Eigen::VectorXd & loghp, size_t nr_iterations)
{
  //// downsample data for HP optimization /////////////////////////////////////
  std::clock_t begin = std::clock();

  // make GP from downsampled data
  libgp::GaussianProcess downsampled_gp(2, "CovSum(CovSEiso, CovNoise)");
  // params, copied from beginning
  // set loghyperparams
  downsampled_gp.covf().set_loghyper(loghp);

  // fill new GP with downsampled data
  if ( m_verbose )
    std::cout << GetAppName() << " :: size m_data_for_hp_optim: " << m_data_for_hp_optim.size() << std::endl;
  double loc[2];

  std::queue< std::vector<double> > temp_queue(m_data_for_hp_optim);
  while ( !temp_queue.empty() )
  {
    std::vector<double> pt = temp_queue.front();
    temp_queue.pop();

    loc[0] = pt[0];
    loc[1] = pt[1];
    double dval = pt[2];
    downsampled_gp.add_pattern(loc, dval);
  }
  std::clock_t end = std::clock();
  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: runtime putting data into downsampled_gp: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;
    std::cout << GetAppName() << " :: size downsampled GP: " << downsampled_gp.get_sampleset_size() << std::endl;
    std::cout << GetAppName() << " :: size orig GP: " << m_gp->get_sampleset_size() << std::endl;
  }

  //// actual HP optimization /////////////////////////////////////
  begin = std::clock();
  // there are 2 methods in gplib, conjugate gradient and RProp,
  // the latter should be more efficient
  if ( m_hp_optim_cg )
  {
    libgp::CG cg;
    cg.maximize(&downsampled_gp, nr_iterations, 0);
  }
  else
  {
    libgp::RProp rprop;
    // RProp arguments: GP, 'n' (nr iterations), verbose
    rprop.maximize(&downsampled_gp, nr_iterations, 1);
  }
  end = std::clock();
  if ( m_verbose )
    std::cout << GetAppName() << " :: runtime hyperparam optimization: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  //// write new HP to file ////////////////////////////////////////////////////
  begin = std::clock();
  std::stringstream filenm;
  filenm << "hp_optim_" << m_db_uptime << "_" << m_veh_name << "_" << nr_iterations;
  downsampled_gp.write(filenm.str().c_str());
  end = std::clock();
  if ( m_debug )
    std::cout << GetAppName() << " :: HP param write to file time: " <<  ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  // downsampled gp should be destroyed
  loghp = downsampled_gp.covf().get_loghyper();
}

//---------------------------------------------------------
// Procedure: getLogGPPredMeanVarFromGPMeanVar
//            convert pred mean/var from GP to pred mean/var for log GP
//
void GP_AUV::getLogGPPredMeanVarFromGPMeanVar(double gp_mean, double gp_cov, double & lgp_mean, double & lgp_cov )
{
  // convert GP mean to log GP mean (lognormal mean)
  lgp_mean = exp(gp_mean + gp_cov/2.0);
  lgp_cov = (lgp_mean*lgp_mean) * (exp(gp_cov) - 1.0);
}

//---------------------------------------------------------
// Procedure: makeAndStorePredictions
//            file writing
//
void GP_AUV::makeAndStorePredictions()
{
  // make a copy of the GP and use that below, to limit lock time
  std::unique_lock<std::mutex> gp_lock(m_gp_mutex, std::defer_lock);
  while ( !gp_lock.try_lock() ) {}
  if ( m_verbose )
    std::cout << GetAppName() << " :: store predictions" << std::endl;
  libgp::GaussianProcess * gp_copy = new libgp::GaussianProcess(*m_gp);
  gp_lock.unlock();

  std::clock_t begin = std::clock();

  std::vector< GraphNode >::iterator loc_itr;
  // get the predictive mean and var values for all sample locations
  std::vector<double> all_pred_means_lGP;
  std::vector<double> all_pred_vars_lGP;
  std::vector<double> all_pred_mu_GP;
  std::vector<double> all_pred_sigma2_GP;
  // pre-alloc vectors
  size_t nr_sample_locations = m_sample_graph_nodes.size();
  all_pred_means_lGP.reserve(nr_sample_locations);
  all_pred_vars_lGP.reserve(nr_sample_locations);
  all_pred_mu_GP.reserve(nr_sample_locations);
  all_pred_sigma2_GP.reserve(nr_sample_locations);

  // make predictions for all sample locations
  for ( loc_itr = m_sample_graph_nodes.begin(); loc_itr < m_sample_graph_nodes.end(); loc_itr++ )
  {
    double loc[2] {loc_itr->get_location()[0], loc_itr->get_location()[1]};
    double pred_mean_GP;
    double pred_var_GP;
    gp_copy->f_and_var(loc, pred_mean_GP, pred_var_GP);

    double pred_mean_lGP, pred_var_lGP;
    if ( m_use_log_gp )
    {
      // params lognormal distr
      // convert to lGP mean and variance
      // to get back in the 'correct' scale for comparison to generated data
      getLogGPPredMeanVarFromGPMeanVar(pred_mean_GP, pred_var_GP, pred_mean_lGP, pred_var_lGP);
    }

    if ( m_use_log_gp )
    {
      all_pred_means_lGP.push_back(pred_mean_lGP);
      all_pred_vars_lGP.push_back(pred_var_lGP);
      all_pred_mu_GP.push_back(pred_mean_GP);
      all_pred_sigma2_GP.push_back(pred_var_GP);
    }
    else
    {
      all_pred_means_lGP.push_back(pred_mean_GP);
      all_pred_vars_lGP.push_back(pred_var_GP);
    }
  }
  std::clock_t end = std::clock();
  if ( m_verbose )
    std::cout << GetAppName() << " :: runtime make predictions: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;

  begin = std::clock();
  // grab file writing mutex
  std::unique_lock<std::mutex> file_write_lock(m_file_writing_mutex, std::defer_lock);
  while ( !file_write_lock.try_lock() ){}
  // write to file
  for ( size_t vector_idx = 0; vector_idx < nr_sample_locations; ++vector_idx )
  {
    // save to file, storing predictions
    if ( vector_idx > 0 )
    {
      m_ofstream_pm_lGP << ", ";
      m_ofstream_pv_lGP << ", ";
      if ( m_use_log_gp )
      {
        m_ofstream_pmu_GP << ", ";
        m_ofstream_psigma2_GP << ", ";
      }
    }
    m_ofstream_pm_lGP << all_pred_means_lGP[vector_idx];
    m_ofstream_pv_lGP << all_pred_vars_lGP[vector_idx];
    if ( m_use_log_gp )
    {
      m_ofstream_pmu_GP << all_pred_mu_GP[vector_idx];
      m_ofstream_psigma2_GP << all_pred_sigma2_GP[vector_idx];
    }
  }
  m_ofstream_pm_lGP << '\n';
  m_ofstream_pv_lGP << '\n';
  if ( m_use_log_gp )
  {
    m_ofstream_pmu_GP << '\n';
    m_ofstream_psigma2_GP << '\n';
  }

  if ( m_finished )
  {
    std::cout << GetAppName() << " :: " << m_db_uptime << " :: closing files." << std::endl;
    m_ofstream_pm_lGP.close();
    m_ofstream_pv_lGP.close();
    if ( m_use_log_gp )
    {
      m_ofstream_pmu_GP.close();
      m_ofstream_psigma2_GP.close();
    }
  }

  file_write_lock.unlock();

  end = std::clock();
  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: runtime save to file: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << std::endl;
    std::cout << GetAppName() << " :: done saving predictions to file" << std::endl;
  }

  // copy of GP gets destroyed when this function exits
  delete gp_copy;
}

//---------------------------------------------------------
// Procedure: calcLonLatSpacing
//            separate out these calculations
//            such that we only do them once
//
void GP_AUV::calcLonLatSpacing()
{
  // convert lane widths in meters to lon/lat
  m_lon_spacing = m_pts_grid_spacing/m_lon_deg_to_m;
  m_lat_spacing = m_pts_grid_spacing/m_lat_deg_to_m;

  // calculate the amount of y vertices
  m_y_resolution = (size_t) round(m_pts_grid_height/m_pts_grid_spacing);
  // add one because of zero indexing
  m_y_resolution++;
}

//---------------------------------------------------------
// Procedure: convLonLatToUTM
//            use MOOS Geodesy to convert lon/lat to x/y
//
bool GP_AUV::convLonLatToUTM (double lon, double lat, double & lx, double & ly )
{
  bool successful = m_geodesy.LatLong2LocalUTM(lat, lon, ly, lx);

  if ( !successful )
    std::cout << GetAppName() << " :: ERROR converting lon/lat to x/y.\n";

  return successful;
}

//---------------------------------------------------------
// Procedure: convUTMToLonLat
//            use MOOS Geodesy to convert x/y to lon/lat
//
bool GP_AUV::convUTMToLonLat (double lx, double ly, double & lon, double & lat )
{
  bool successful = m_geodesy.UTM2LatLong(lx, ly, lat, lon);

  if ( !successful )
    std::cout << GetAppName() << " :: ERROR converting x/y to lon/lat.\n";

  return successful;
}


//---------------------------------------------------------
// Procedure: inSampleRectangle(double veh_lon, double veh_lat, bool use_buffer) const
//            see if the current vehicle position is inside the
//            prespecified sampling area (rectangle)
//
bool GP_AUV::inSampleRectangle(double veh_lon, double veh_lat, bool use_buffer) const
{
  // if just outside data grid, but close enough,
  // should be able to map to border points
  double buffer_lon = m_area_buffer/m_lon_deg_to_m;
  double buffer_lat = m_area_buffer/m_lat_deg_to_m;

  if ( use_buffer )
    return ( veh_lon >= (m_min_lon-buffer_lon) && veh_lat >= (m_min_lat-buffer_lat) &&
             veh_lon <= (m_max_lon+buffer_lon) && veh_lat <= (m_max_lat+buffer_lat) );
  else
    return ( veh_lon >= m_min_lon && veh_lat >= m_min_lat &&
             veh_lon <= m_max_lon && veh_lat <= m_max_lat );
}


//---------------------------------------------------------
// Procedure: tdsResetStateVars
//            reset state vars for TDS control
//
void GP_AUV::tdsResetStateVars()
{
  if ( m_verbose )
    std::cout << GetAppName() << " :: reset state vars" << std::endl;

  // move to next step; need wpt
  m_mission_state = STATE_SAMPLE;
  publishStates("tdsResetStateVards_else");

  if ( m_bhv_state != "survey" )
    Notify("STAGE","survey");

}

//---------------------------------------------------------
// Procedure: findAndPublishNextWpt
//            we need to calculate the next waypoints,
//            and publish to MOOSDB when found
//            because the calculations are heavy, we start
//            a thread here.
//
void GP_AUV::findAndPublishNextWpt()
{
  if ( !m_finding_nxt )
  {
    if ( m_verbose )
      std::cout << GetAppName() << " :: calling to find next sample location" << std::endl;
    kickOffCalcMetric();
  }
  else
  {
    // see if we can get result from future, which was created via kickOffCalcMetric
    std::cout << GetAppName() << " :: checking future m_future_next_pt" << std::endl;
    if ( m_future_next_pt.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
    {
      m_finding_nxt = false;

      // done with maximum entropy calculations

      // now call functions for path planning
      // called from inside publishNextWaypointLocations method
      publishNextWaypointLocations();

      // continue survey
      if ( m_bhv_state != "survey" )
        m_Comms.Notify("STAGE","survey");
      m_mission_state = STATE_SAMPLE;
      // publish states for debugging/eval purposes
      publishStates("findAndPublishNextWpt");
    }
  }
}

//---------------------------------------------------------
// Procedure: ownMessage(std::string input)
//            check if this message is from own vehicle
//
bool GP_AUV::ownMessage(std::string input)
{
  size_t index = input.find(m_veh_name);
  return ( index != std::string::npos );
}

//---------------------------------------------------------
// Procedure: finalSurface(std::string input)
//            check if message contains word 'final'
//
bool GP_AUV::finalSurface(std::string input)
{
  // note; we give value 'final', but surface request is a bool
  // given message size, so it would be converted to 'false'
  size_t index = input.find("false");
  return ( index != std::string::npos );
}


void GP_AUV::publishStates(std::string const calling_method)
{
  if ( m_verbose )
    std::cout << GetAppName() << " :: ** " << m_db_uptime << " switch to: " << currentMissionStateString() << " from " << calling_method << " **\n";

  m_Comms.Notify("STATE_MISSION", currentMissionStateString());
}

const GraphNode* GP_AUV::max(const GraphNode* a, const GraphNode* b, const GraphNode* c)
//GraphNode* GP_AUV::max(GraphNode* a, GraphNode* b, GraphNode* c)
{
    double aVal = (a == nullptr ? INT16_MIN : a->get_value());
    double bVal = (b == nullptr ? INT16_MIN : b->get_value());
    double cVal = (c == nullptr ? INT16_MIN : c->get_value());
    std::cout << GetAppName() << " :: a:" << aVal << " :: b:" << bVal << " :: c:" << cVal << std::endl;

    if(aVal >= bVal && aVal >= cVal) {
        std::cout << GetAppName() << " :: selected a" << std::endl;
        return a;
    }
    else if(bVal >= aVal && bVal >= cVal) {
        std::cout << GetAppName() << " :: selected b" << std::endl;
        return b;
    }
    else if(cVal >= aVal && cVal >= aVal) {
        std::cout << GetAppName() << " :: selected c" << std::endl;
        return c;
    }
}
