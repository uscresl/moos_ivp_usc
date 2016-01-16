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


//---------------------------------------------------------
// Constructor
//
GP::GP() :
  m_gp(2, "CovSum(CovSEiso, CovNoise)")
{
  // class variable instantiations can go here
  m_input_var = "";

  m_lat = 0;
  m_lon = 0;
  m_dep = 0;
  m_data_added = false;

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
    
    if( key == m_input_var )
    {
      //handleMailGPVarIn(sval);
      std::cout << "\nreceiving: " << dval << " " << atof( sval.c_str() ) << std::endl;
      handleMailData(dval);
    }
    else if ( key == "NAV_LAT" )
      m_lat = dval;
    else if ( key == "NAV_LONG" )
      m_lon = dval;
    else if ( key == "NAV_DEPTH" )
      m_dep = dval;
    else
      std::cout << "pGP :: Unhandled Mail: " << key << std::endl;
    //reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool GP::OnConnectToServer()
{
  registerVariables();
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
    // predict target value for given input, f()
    // predict variance of prediction for given input, var()
    double x_t[] = {m_lon+0.00001, m_lat+0.00001}; //, m_dep+0.1}; //TODO change/move
    double pred_f = m_gp.f(x_t);
    double pred_var = m_gp.var(x_t);

    std::cout << "\n";
    std::cout << "pred_f: " << pred_f << '\n';
    std::cout << "pred_var: " << pred_var << std::endl;
  }

  std::cout << m_gp.get_sampleset_size() << std::endl;

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
  //reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p=sParams.begin(); p!=sParams.end(); p++)
  {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if ( param == "input_var" )
    {
      m_input_var = toupper(value);
      handled = true;
    }

    if(!handled)
      std::cout << GetAppName() << " :: Unhandled Config: " << orig << std::endl;
    //reportUnhandledConfigWarning(orig);
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
  if ( m_input_var != "" )
    m_Comms.Register(m_input_var, 0);
}

//---------------------------------------------------------
// Procedure: handleMailData
//            handle the incoming message
//
bool GP::handleMailData(double received_data)
{
  if ( m_lon == 0 && m_lat == 0 && m_dep == 0 )
  {
    std::cout << "No NAV_LAT/LON/DEPTH received, not processing data." << std::endl;
    return false;
  }
  else
  {
    // Parse and handle ack message components
    bool   valid_msg = true;

    // add training data
    // Input vectors x must be provided as double[] and targets y as double.
    double x[] = {m_lon, m_lat}; //, m_dep};
    m_gp.add_pattern(x, received_data);
    m_data_added = true;

    return ( valid_msg );
  }
  return false;
}
