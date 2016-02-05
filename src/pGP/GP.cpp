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

//---------------------------------------------------------
// Constructor
//
GP::GP() :
  m_gp(2, "CovSum(CovSEiso, CovNoise)")
{
  // class variable instantiations can go here
  m_input_var_data = "";
  m_input_var_sample_points = "";
  m_input_var_sample_points_specs = "";
  m_output_var_pred = "";
  m_prediction_interval = -1;

  m_lat = 0;
  m_lon = 0;
  m_dep = 0;
  m_data_added = false;
  m_last_published = std::numeric_limits<double>::max();

  // initialize a GP for 2D input data, //TODO convert to 3D
  // using the squared exponential covariance function,
  // with additive white noise

  // Set log-hyperparameter of the covariance function.
  Eigen::VectorXd params(m_gp.covf().get_param_dim());
  // hyperparameters: length scale l^2, signal variance s_f^2, noise variance s_n^2
  // note, these can be optimized using cg or rprop
  params << 0.0, 0.0, -2.0;
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
      // process sample locations
      storeSamplePoints(sval);
    }
    else if ( key == m_input_var_sample_points_specs )
    {
      // process specs for sample points
      storeSamplePointsSpecs(sval);
    }
    else
      std::cout << "pGP :: Unhandled Mail: " << key << std::endl;
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool GP::OnConnectToServer()
{
//  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second
bool GP::Iterate()
{
  if ( m_lon == 0 && m_lat == 0 && m_dep == 0 )
    return true;
  else if ( m_data_added )
  {
    // predict target value and variance for sample locations
    if ( (size_t)std::floor(MOOSTime()) % m_prediction_interval == 0  && (std::abs(m_last_published - MOOSTime()) > 1) ) // every 5 min, for now
    {
      std::clock_t begin = std::clock();
      findNextSampleLocation();
      std::clock_t end = std::clock();
      std::cout << "runtime findNextSampleLocation: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << '\n' << std::endl;
    }

    // update sample sets
    updateVisitedSet();
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
    // add training data
    // Input vectors x must be provided as double[] and targets y as double.
    double x[] = {m_lon, m_lat}; //, m_dep};
    m_gp.add_pattern(x, received_data);
    m_data_added = true;
  }
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
    Eigen::VectorXd loc_vec(2);
    loc_vec(0) = lon;
    loc_vec(1) = lat;
    m_sample_points_unvisited.insert( std::pair<size_t, Eigen::VectorXd>(id_pt, loc_vec) );

    // the first location should be bottom left corner, store as minima
    if ( id_pt == 0 )
    {
      m_min_lon = lon;
      m_min_lat = lat;
      std::cout << GetAppName() << " :: SW Sample location: " << std::setprecision(10) << lon << " " << lat << std::endl;
    }
  }
  // check / communicate what we did
  std::cout << GetAppName() << " :: stored " << m_sample_points_unvisited.size() << " sample locations" << std::endl;
}

//---------------------------------------------------------
// Procedure: storeSamplePoints
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
  // tmp
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
  // store values in tmp to avoid change while this procedure is run
  double veh_lon = m_lon;
  double veh_lat = m_lat;

  // TODO move this to library?
  double lat_deg_to_m = 110923.99118801417;
  double lon_deg_to_m = 92287.20804979937;
  double lon_spacing = m_pts_grid_spacing/lon_deg_to_m; // convert lane width to lon
  double lat_spacing = m_pts_grid_spacing/lat_deg_to_m; // convert lane width to lat

  // TODO: boundary conditions
  // if just outside data grid, should be able to map to border points, if close enough
  // currently we only have min lat/lon stored, need also max, and then check for all boundaries

  // calculate the id of the location where the vehicle is currently
  // at, and move it to visited
  if ( veh_lon >= m_min_lon && veh_lat >= m_min_lat )
  {
    double x_cell = (veh_lon - m_min_lon)/lon_spacing;
    double y_cell = (veh_lat - m_min_lat)/lat_spacing;
    size_t x_cell_rnd = (size_t) round(x_cell);
    size_t y_cell_rnd = (size_t) round(y_cell);
    size_t y_resolution = (size_t) round(m_pts_grid_height/m_pts_grid_spacing);
    // add one because of zero indexing
    y_resolution++;
    // calculate index into map (stored from SW, y first, then x)
    size_t index = y_resolution*x_cell_rnd + y_cell_rnd;

    std::map<size_t, Eigen::VectorXd>::iterator curr_loc_itr = m_sample_points_unvisited.find(index);
    if ( curr_loc_itr != m_sample_points_unvisited.end() )
    {

      std::clock_t begin = std::clock();

      // remove point from unvisited set
      Eigen::VectorXd move_pt = m_sample_points_unvisited.at(index);

      // tmp check
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

      // now remove it from the unvisited set
      m_sample_points_unvisited.erase(curr_loc_itr);

      // and add the point to the visited set
      m_sample_points_visited.insert(std::pair<size_t, Eigen::VectorXd>(index, move_pt));

      // report
      std::cout << "moved pt: " << std::setprecision(10) << move_pt(0) << ", " << move_pt(1);
      std::cout << " from unvisited to visited.\n";
      std::cout << "Unvisited size: " << m_sample_points_unvisited.size() << '\n';
      std::cout << "Visited size: " << m_sample_points_visited.size() << std::endl;

      std::clock_t end = std::clock();
      std::cout << "runtime updateVisitedSet: " << ( (double(end-begin) / CLOCKS_PER_SEC) ) << '\n'<< std::endl;
    }
  }
  else
  {
//    // boundary conditions
//    std::cout << GetAppName() << " :: current vehicle location outside data grid\n";
//    std::cout << GetAppName() << " :: lon boundary ok? " << ( veh_lon >= m_min_lon ? "yes" : "no" ) << '\n';
//    std::cout << GetAppName() << " :: lat boundary ok? " << ( veh_lat >= m_min_lat ? "yes" : "no" ) << '\n';
    return;
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
  libgp::CovarianceFunction & cov_f = m_gp.covf();

  // for each y (from unvisited set only, as in greedy algorithm Krause'08)
  // calculate the mutual information term
  size_t size_visited = m_sample_points_visited.size();
  size_t size_unvisited = m_sample_points_unvisited.size();

  if ( size_visited > 0 )
  {

    // calculate covariance matrices sets, and their inverses (costly operations)
    Eigen::MatrixXd K_aa(size_visited, size_visited);
    createCovarMatrix(cov_f, "visited", K_aa);
    Eigen::MatrixXd K_aa_inv = K_aa.inverse();

    Eigen::MatrixXd K_avav(size_unvisited, size_unvisited);
    createCovarMatrix(cov_f, "unvisited", K_avav);
    Eigen::MatrixXd K_avav_inv = K_avav.inverse();

    double best_so_far = std::numeric_limits<double>::min();
    Eigen::VectorXd best_so_far_y(2);
    std::map<size_t, Eigen::VectorXd>::iterator y_itr;
    for ( y_itr = m_sample_points_unvisited.begin(); y_itr != m_sample_points_unvisited.end(); y_itr++ )
    {
      Eigen::VectorXd y(2);
      y = y_itr->second;

      // calculate k(y,y)
      double k_yy = cov_f.get(y,y);

      // calculate covariance with visited set
      Eigen::VectorXd k_ya(size_visited);
      createCovarVector(cov_f, y, "visited", k_ya);
      // TODO: add noise term?
      double mat_ops_result = k_ya.transpose() * K_aa_inv * k_ya;
      double sigma_y_A = k_yy - mat_ops_result;

      // calculate covariance with unvisited set
      Eigen::VectorXd k_yav(size_unvisited);
      createCovarVector(cov_f, y, "unvisited", k_yav);
      // TODO add noise term?
      mat_ops_result = k_yav.transpose() * K_avav_inv * k_yav;
      double sigma_y_Av = k_yy - mat_ops_result;

      // calculate mutual information term
      double div = 0.5 * log(sigma_y_A / sigma_y_Av);

      // store max
      if ( div > best_so_far )
      {
        best_so_far = div;
        best_so_far_y = y;
      }
    }

    std::ostringstream output_stream;
    output_stream << "best_y=" << std::setprecision(15) << best_so_far_y(0) << "," << best_so_far_y(1);
    // TODO, use this for figuring out where to go next
    m_Comms.Notify(m_output_var_pred, output_stream.str());
    std::cout << GetAppName() << " :: publishing " << m_output_var_pred << std::endl;
    m_last_published = MOOSTime();
  }
}

//---------------------------------------------------------
// Procedure: createCovarVector
//            helper method for mutual information calculation
//            calculates K_ya
//
void GP::createCovarVector(libgp::CovarianceFunction& cov_f, Eigen::VectorXd y, std::string const & set_identifier, Eigen::VectorXd & k_ya)
{
  // choose which map to use
  std::map<size_t, Eigen::VectorXd> & map_ref = ( set_identifier == "visited" ? m_sample_points_visited : m_sample_points_unvisited);

  // calculate the covariances
  std::map<size_t, Eigen::VectorXd>::iterator a_itr;
  size_t a_cnt = 0;
  for ( a_itr = map_ref.begin(); a_itr != map_ref.end(); a_itr++, a_cnt++)
  {
    // calc k_ya
    Eigen::VectorXd a(2);
    a = a_itr->second;

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
  std::map<size_t, Eigen::VectorXd> & map_ref = ( set_identifier == "visited" ? m_sample_points_visited : m_sample_points_unvisited);

  std::map<size_t, Eigen::VectorXd>::iterator a_itr;
  size_t a_cnt = 0;
  for ( a_itr = map_ref.begin(); a_itr != map_ref.end(); a_itr++, a_cnt++)
  {
    Eigen::VectorXd a(2);
    a = a_itr->second;

    // calc K_aa
    std::map<size_t, Eigen::VectorXd>::iterator b_itr;
    size_t b_cnt = 0;
    for ( b_itr = map_ref.begin(); b_itr != map_ref.end(); b_itr++, b_cnt++ )
    {
      Eigen::VectorXd b(2);
      b = b_itr->second;

      K_aa(a_cnt, b_cnt) = cov_f.get(a, b);
    }
  }
}
