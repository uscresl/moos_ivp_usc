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

//---------------------------------------------------------
// Constructor
//
GP::GP() :
  m_gp(2, "CovSum(CovSEiso, CovNoise)")
{
  // class variable instantiations can go here
  m_input_var_data = "";
  m_input_var_sample_points = "";
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
      // predict target value for given input, f()
      // predict variance of prediction for given input, var()

      //    double x_t[] = {m_lon+0.00001, m_lat+0.00001}; //, m_dep+0.1}; //TODO change/move
      //    double pred_f = m_gp.f(x_t);
      //    double pred_var = m_gp.var(x_t);
      std::ostringstream output_stream;
      for ( size_t loc_idx = 0; loc_idx < m_sample_points.size(); loc_idx++ )
      {
        std::pair<double, double> location = m_sample_points.at(loc_idx);
        double x_t[] = {location.first, location.second};
        double pred_f = m_gp.f(x_t);
        double pred_var = m_gp.var(x_t);
        output_stream << pred_f << "," << pred_var << ";";
      }

      // TODO, use this for figuring out where to go next
      m_Comms.Notify(m_output_var_pred, output_stream.str());
      std::cout << GetAppName() << " :: publishing " << m_output_var_pred << std::endl;
      m_last_published = MOOSTime();

      // TESTING 123 TODO: change to use sampled/unsampled locations
      // get covariance function from GP
      // have two sets; sampled and unsampled locations
      // use the get() function from the CovarianceFunction
      libgp::CovarianceFunction & cov_f = m_gp.covf();
      std::cout << "covar func: " << cov_f.to_string() << std::endl;
      Eigen::VectorXd y(2);
      y(0) = -117.806000;
      y(1) = 34.088000;

      Eigen::VectorXd sampled_locations(2);
      sampled_locations(0) = -117.809000;
      sampled_locations(1) = 34.080000;
      double k_ya = cov_f.get(y, sampled_locations);
      double k_ay = cov_f.get(sampled_locations, y);
      std::cout << " covariance: " << k_ya << "  " << k_ay << std::endl;
      double k_yy = cov_f.get(y, y);
      std::cout << "cov yy : " << k_yy << std::endl;

//      double k_yy = cov_f.get(y, y);
//      double k_yA = cov_f.get(y, sampled_locations);
//      double k_Ay = cov_f.get(sampled_locations, y);
//      double k_AA = cov_f.get(sampled_locations, sampled_locations);
//      double k_yAb = cov_f.get(y, unsampled_locations);
//      double k_Aby = cov_f.get(y, sampled_locations);
    }
  }

//  std::cout << m_gp.get_sampleset_size() << std::endl;

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

    bool handled = false;
    if ( param == "input_var_data" )
    {
      m_input_var_data = toupper(value);
      handled = true;
    }
    else if ( param == "input_var_sample_points" )
    {
      m_input_var_sample_points = toupper(value);
      handled = true;
    }
    else if ( param == "output_var_predictions" )
    {
      m_output_var_pred = toupper(value);
      handled = true;
    }
    else if ( param == "prediction_interval" )
    {
      m_prediction_interval = (size_t)atoi(value.c_str());
      if ( m_prediction_interval < 0 )
      {
        std::cout << GetAppName() << " :: ERROR, invalid prediction interval, needs to be > 0" << std::endl;
        RequestQuit();
      }
      else
        handled = true;
    }

    if(!handled)
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
    m_sample_points.push_back( std::pair<double, double>(lon, lat) );
  }
  // check / communicate what we did
  std::cout << GetAppName() << " :: stored " << m_sample_points.size() << " sample locations" << std::endl;
}
