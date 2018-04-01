/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: GP.cpp                                               */
/*    DATE: 2015 - 2016                                          */
/*                                                               */
/*****************************************************************/

#include "PGP.h"

#include <iterator>
#include "MBUtils.h"

#include "math.h"
#include <limits>

#include <boost/lexical_cast.hpp>

#include <cov.h>
#include <cov_se_iso.h>

// check running time
#include <time.h>

// GPLib Rprop
#include "rprop.h"
#include "cg.h"

// write to file
#include <fstream>

// init srand
#include <chrono>


//---------------------------------------------------------
// Constructor
//
GP::GP() :
  m_verbose(true),
  m_input_var_data(""),
  m_input_var_sample_points(""),
  m_input_var_sample_points_specs(""),
  m_input_var_adaptive_trigger(""),
  m_input_var_share_data(""),
  m_output_var_pred(""),
  m_output_filename_prefix(""),
  m_output_var_share_data(""),
  m_prediction_interval(-1),
  m_max_wait_for_other_vehicles(120),
  m_debug(true),
  m_veh_name(""),
  m_use_log_gp(true),
  m_lat(0),
  m_lon(0),
  m_dep(0),
  m_surf_cnt(0),
  m_at_depth_cnt(0),
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
  m_hp_dev_ratio(0.25),
  m_last_hp_optim_done(0),
  m_data_mail_counter(1),
  m_finished(false),
  m_num_vehicles(1),
  m_data_pt_counter(0),
  m_data_send_reserve(0),
  m_received_shared_data(false),
  m_data_received(false),
  m_timed_data_sharing(false),
  m_data_sharing_interval(600),
  m_waiting(false),
  m_received_ready(false),
  m_output_var_handshake_data_sharing(""),
  m_last_ready_sent(0),
  m_handshake_timer_counter(0),
  m_tx_timer_counter(0),
  m_rx_timer_counter(0),
  m_req_surf_timer_counter(0),
  m_last_surface(0),
  m_acomms_sharing(false),
  m_last_acomms_string(""),
  m_use_voronoi(false),
  m_running_voronoi_routine(false),
  m_calc_prevoronoi(false),
  m_precalc_pred_voronoi_done(false),
  m_vor_timeout(300),
  m_downsample_factor(4),
  m_first_surface(true),
  m_nr_ack_sent(0),
  m_area_buffer(5.0),
  m_bhv_state(""),
  m_adp_state(""),
  m_use_surface_hub(false),
  m_veh_is_shub(false),
  m_cancel_hpo(false),
  m_prev_length_scale(0),
  m_async_trigger_method("timed"),
  m_async_threshold(0.3),
  m_async_prev_sum_var(0.0),
  m_async_prev_sum_var_reset(true),
  m_need_to_run_hpo(false),
  m_survey_depth(0.0),
  m_survey_speed(0.0),
  m_dive_pitch_angle(0.0),
  m_twoway_time_to_surf(0.0),
  m_mission_duration(3600.0),
  m_calcwpt_from_data_sharing(false)
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
  // hyperparameters: length scale l, signal variance s_f^2, noise variance s_n^2
  // note, these will be optimized using cg or rprop
  // length scale: avg lat/lon_deg_to_m is 100000, 10m range = 0.0001
  //               20m: 0.00020, ln() = -8.5
  //               55m: 0.00055, ln() = -7.5 <- similar to field results
  // signal: from 0 to ca 30/40
  //         stdev 4.0, ln() = 1.38
  // noise: let's set 10x smaller than signal
  //        stdev 0.4, ln() = -0.91
  params << -7.5, 1.38, -0.92;
  m_gp->covf().set_loghyper(params);
  m_prev_length_scale = params(0);

  // use a unique seed to initialize srand,
  // using milliseconds because vehicles can start within same second
  struct timeval time;
  gettimeofday(&time,NULL);
  int rand_seed = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  srand(rand_seed);

#ifdef BUILD_VORONOI
  m_use_voronoi = true;
#else
  m_use_voronoi = false;
#endif
}

GP::~GP()
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
bool GP::OnNewMail(MOOSMSG_LIST &NewMail)
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

      // downsample data, published at frequency of 2 Hz
      if ( (m_data_mail_counter % 2 == 0) &&
            m_mission_state == STATE_SAMPLE )
        handleMailDataFromSensor(dval);
    }
    else if ( key == "NAV_LAT" )
      m_lat = dval;
    else if ( key == "NAV_LONG" )
      m_lon = dval;
    else if ( key == "NAV_DEPTH" )
    {
      m_dep = dval;

      if ( std::abs(m_dep) < 0.2 )
      {
        m_surf_cnt++;
        m_at_depth_cnt = 0;
      }
      else
      {
        m_surf_cnt = 0;
        m_at_depth_cnt++;
      }

      if ( m_surf_cnt > 10 )
        m_on_surface = true;
      else if ( m_at_depth_cnt > 5 )
        m_on_surface = false;
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
    else if ( key == m_input_var_share_data )
    {
      // handle
      std::string incoming_data_string = sval;
      size_t index_colon = incoming_data_string.find_first_of(':');
      std::string veh_nm = incoming_data_string.substr(0, index_colon);
      std::string remainder = incoming_data_string.substr(index_colon+1);
      size_t index_colon2 = remainder.find_first_of(':');
      std::string destination_veh = remainder.substr(0, index_colon2);
      if ( veh_nm == m_veh_name )
        std::cout << GetAppName() << " :: skipping my own data" << std::endl;
      else if ( destination_veh != m_veh_name )
        std::cout << GetAppName() << " :: data is not meant for me (" << m_veh_name
                  << "), but for: " << destination_veh << std::endl;
      else
      {
        if ( m_verbose )
          std::cout << GetAppName() << " :: received data from " << veh_nm << std::endl;
        if ( !m_on_surface && m_verbose )
          std::cout << GetAppName() << " :: not on surface, skipping data." << std::endl;

        if ( m_use_surface_hub && !m_veh_is_shub && veh_nm != "shub" && m_on_surface )
        {
          // if the simulation works with a surface hub, we explicitly ignore
          // surface-based communications coming from other AUVs
          std::cout << GetAppName() << " :: using surface hub, skip data from others" << std::endl;
        }
        else if ( m_on_surface ) // avoid adding data meant for other vehicles while underwater
        {
          m_received_shared_data = true;

          // extract actual data
          // if we are working with the surface hub, pass on the name as well
          std::string incoming_data;
          if ( m_use_surface_hub && m_veh_is_shub )
            incoming_data = incoming_data_string;
          else
            incoming_data = remainder.substr(index_colon2+1);

          if ( m_debug )
            std::cout << GetAppName() << " :: adding data from: " << veh_nm
                      << " to destination: " << destination_veh << std::endl;

          m_incoming_data_to_be_added.push_back(incoming_data);
          m_data_received = true;

          if ( m_final_hp_optim && m_veh_is_shub )
          {
            if ( std::find(m_final_received_from.begin(), m_final_received_from.end(), veh_nm) == m_final_received_from.end() )
            {
              m_final_received_from.push_back(veh_nm);
              std::cout << GetAppName() << " :: m_final_received_cnt = "
                        << m_final_received_from.size() << std::endl;
              std::cout << GetAppName() << " :: increased by data from: "
                        << veh_nm << " at: " << currentMOOSTime() << std::endl;
            }
            else
              std::cout << GetAppName() << " :: already counted: " << veh_nm
                        << " at: " << currentMOOSTime() << std::endl;
          }
        }
      }
    }
    else if ( key == "LOITER_DIST_TO_POLY" )
      m_loiter_dist_to_poly = dval;
    else if ( key == m_input_var_handshake_data_sharing )
    {
      std::string veh_that_is_ready = sval;
      if ( veh_that_is_ready != m_veh_name )
      {
        if ( m_on_surface ) // simulate that we only receive on surface
        {
          if ( m_verbose )
            std::cout << GetAppName() << " :: received READY at: " << currentMOOSTime() << " from: " << sval << std::endl;

          // check if vehicle not in list yet, if so, add
          if ( m_rec_ready_veh.find(sval) == m_rec_ready_veh.end() )
            m_rec_ready_veh.insert(sval);

          // if we have received 'ready' from as many vehicles as exist, then flip bool
          if ( m_debug )
          {
            std::cout << GetAppName() << " :: m_rec_ready_veh.size(), m_other_vehicles.size(): "
                      << m_rec_ready_veh.size() << ", " << m_other_vehicles.size() << std::endl;


            std::cout << GetAppName() << " :: m_use_surface_hub, m_rec_ready_veh.size() > 0? "
                      << m_use_surface_hub << ", " << (m_rec_ready_veh.size() > 0) << std::endl;
          }

          // cases:
          // 1. no shub, not VOR (req_surf): if all received, set received_ready
          // 2. no shub, VOR (req_surf): if all received, set received_ready
          //                             and switch to HANDSHAKE
          // 3. w/ shub, not VOR: set received_ready, store who is ready
          // 4. w/ shub, VOR: if all received, set received_ready  (first if)
          //
          if ( ( m_rec_ready_veh.size() == m_other_vehicles.size() &&
                 m_mission_state == STATE_HANDSHAKE && !m_use_surface_hub ) ||
               ( m_veh_is_shub && m_use_voronoi && m_rec_ready_veh.size() == m_other_vehicles.size() ) )
          {
            for ( auto veh : m_rec_ready_veh )
              std::cout << GetAppName() << " :: received ready from: "
                        << veh << " at: " << currentMOOSTime() << std::endl;
            if ( !m_received_ready )
              m_received_ready = true;

            if ( m_veh_is_shub && m_use_voronoi )
            { // continue!
              m_mission_state = STATE_HANDSHAKE;
              publishStates("OnNewMail_m_input_var_handshake_data_sharing_shub");
            }
          }
          else if ( m_rec_ready_veh.size() == m_other_vehicles.size() &&
                    m_mission_state == STATE_REQ_SURF &&
                    !m_use_surface_hub )
          {
            if ( m_debug )
              std::cout << GetAppName() << " :: received READY from both vehicles "
                        << "while on surface, though still in REQ_SURF."
                        << "Switch to handshake." << std::endl;
            m_received_ready = true;
            m_mission_state = STATE_HANDSHAKE;
            publishStates("OnNewMail_m_input_var_handshake_data_sharing");
          }
          else if ( m_use_surface_hub &&
                    m_rec_ready_veh.size() > 0 &&
                    (!m_use_voronoi || !m_veh_is_shub) )
          {
            // surface hub case:
            // both shub and vehicle don't need to wait for all vehicles to be ready
            // just the other in the exchange should be ready
            if ( !m_received_ready )
            {
              // for the AUVs, want to make sure that surface hub is ready
              if ( (!m_veh_is_shub && veh_that_is_ready == "shub" &&
                    (m_mission_state == STATE_SURFACING || m_mission_state == STATE_HANDSHAKE)) ||
                    m_veh_is_shub )
              {
                // set var to continue with handshake
                m_received_ready = true;

                if ( m_veh_is_shub && m_mission_state == STATE_SAMPLE )
                {
                  // get ready to receive and send data: go to handshake mode
                  m_last_surface = MOOSTime();
                  m_mission_state = STATE_SURFACING;
                  publishStates("Incoming_handshake_all_received_surface_hub");
                }
              }
              else
              {
                if ( m_debug )
                  std::cout << GetAppName() << " :: received ready from: "
                            << veh_that_is_ready << ", at: " << currentMOOSTime()
                            << " skipping." << std::endl;
              }
            }
            else
            {
              if ( m_debug )
              {
                std::cout << GetAppName() << " :: received ready from: " << veh_that_is_ready
                          << ", but m_received_ready, skipping message." << std::endl;
              }
            }
          }
        }
        else
        {
          if ( m_debug )
            std::cout << GetAppName() << " :: received ready but not at surface yet, skipping" << std::endl;
        }
      }
      else
      {
        if ( m_debug )
          std::cout << GetAppName() << " :: received ready from self, skipping" << std::endl;
      }
    }
    else if ( key == "INCOMING_DATA_ACOMMS" )
    {
      handleMailDataAcomms(sval);
    }
    else if ( key == "NODE_REPORT")
    {
      handleMailNodeReports(sval);
    }
    else if ( key == "TEST_VORONOI" )
    {
      if ( m_debug )
        std::cout << GetAppName() << " :: calcVoronoi requested by TEST_VORONOI." << std::endl;
      #ifdef BUILD_VORONOI
      calcVoronoi(m_lon, m_lat, m_other_vehicles);
      #endif
    }
    else if ( key == "STAGE" )
      m_bhv_state = sval;
    else if ( key == "ADP_PTS" )
    {
      m_adp_state = sval;
      if ( m_adp_state == "adaptive" && m_timed_data_sharing && !m_use_voronoi )
      {
        std::cout << GetAppName() << " :: done with static survey, initiate surfacing for TDS" << std::endl;
        // switch to surface
        m_last_surface = MOOSTime();
        m_mission_state = STATE_SURFACING;
        publishStates("OnNewMail_ADP_PTS");
      }
    }
    else if ( key == "REQ_SURFACING_REC" )
    { // receive surfacing request from other vehicle
      // need to send ack, also start surfacing
      bool own_msg = ownMessage(sval);
      if ( m_debug )
      {
        std::cout << GetAppName() << " :: REQ_SURFACING_REC own msg? "
                  << own_msg << " at: " << currentMOOSTime() <<  '\n';
        std::cout << GetAppName() << " :: processing msg? "
                  << ( (m_mission_state == STATE_SAMPLE ||
                        m_mission_state == STATE_CALCWPT ||
                        m_mission_state == STATE_REQ_SURF) &&
                       ((MOOSTime()-m_last_voronoi_calc_time) > m_vor_timeout) );
        if ( !(m_mission_state == STATE_SAMPLE ||
               m_mission_state == STATE_CALCWPT ||
               m_mission_state == STATE_REQ_SURF) ||
             !((MOOSTime()-m_last_voronoi_calc_time) > m_vor_timeout) )
          std::cout << GetAppName() << " :: because: (SAMPLE || CALCWPT || REQ_SURF)? " <<
                    (m_mission_state == STATE_SAMPLE || m_mission_state == STATE_CALCWPT || m_mission_state == STATE_REQ_SURF)
                    << ", or: (MOOSTime()-m_last_voronoi_calc_time) > m_vor_timeout? " <<
                    ((MOOSTime()-m_last_voronoi_calc_time) > m_vor_timeout)
                    << std::endl;
      }
      if ( m_adp_state == "static" )
        std::cout << GetAppName() << " :: REQ_SURFACING_REC - but running static pts, ignore for now" << std::endl;


      if ( !own_msg && m_adp_state != "static" )
      {
        bool final_surface = finalSurface(sval);
        std::cout << GetAppName() << " :: Received surface request for final surface? " << final_surface << std::endl;

        if ( ((MOOSTime()-m_last_voronoi_calc_time) > m_vor_timeout) || final_surface )
        {
          if ( m_mission_state == STATE_SAMPLE ||
               m_mission_state == STATE_CALCWPT )
          {
            // start to surface (bhv)
            if ( m_bhv_state != "data_sharing" )
              Notify("STAGE","data_sharing");
          }
          if ( m_mission_state == STATE_REQ_SURF )
          {
            // start to surface (bhv)
            if ( m_bhv_state != "data_sharing" && !m_final_hp_optim )
              Notify("STAGE","data_sharing");
          }
          if ( m_mission_state == STATE_SAMPLE ||
               m_mission_state == STATE_CALCWPT ||
               m_mission_state == STATE_REQ_SURF )
          {
            // prep for surfacing
            // send ack, do actual sending in Iterate so we send until surface handshake
            m_mission_state = STATE_ACK_SURF;
            publishStates("OnNewMail_REQ_SURFACING_REC");
          }
          if ( final_surface )
          {
            m_final_hp_optim = true;

            if ( m_hp_optim_running )
            {
              if ( m_debug )
                std::cout << GetAppName() << " :: REQ_SURF finale surface,"
                          << " resetting m_calc_prevoronoi" << std::endl;
              m_calc_prevoronoi = false;
              m_cancel_hpo = true;
              if ( m_debug )
                std::cout << GetAppName() << " :: setting m_calc_prevoronoi "
                          << "to false, final surface req msg" << std::endl;
            }
            if ( m_mission_state == STATE_SAMPLE ||
                 m_mission_state == STATE_CALCWPT ||
                 m_mission_state == STATE_REQ_SURF )
            {
              if ( m_bhv_state != "hpoptim" )
                m_Comms.Notify("STAGE","hpoptim");
            }
          }
        }
      }
    }
    else if ( key == "REQ_SURFACING_ACK_REC" )
    { // receive surfacing req ack from other vehicle
      bool own_msg = ownMessage(sval);
      if ( m_debug )
        std::cout << GetAppName() << " :: REQ_SURFACING_ACK_REC own msg? "
                  << own_msg << '\n';

      std::string vname;
      bool getval = MOOSValFromString(vname, sval, "veh", true);
      if ( getval )
      {
        // count the nr of acks received; need from all vehicles
        if ( m_rec_ack_veh.find(vname) == m_rec_ack_veh.end() )
          m_rec_ack_veh.insert(vname);
      }

      if ( m_debug )
      {
        std::cout << GetAppName() << " :: ACK received from: " << vname << std::endl;
        std::cout << GetAppName() << " :: processing msg? surf_req: " << ( m_mission_state == STATE_REQ_SURF )
                  << " nr_veh ok? " << (m_rec_ack_veh.size() == m_other_vehicles.size()) << std::endl;
      }

      // change state if ack received from all other vehicles
      if ( !own_msg && m_mission_state == STATE_REQ_SURF &&
           m_rec_ack_veh.size() == m_other_vehicles.size() )
      {
        // prep for surfacing
        // ack received, start surfacing
        m_last_surface = MOOSTime();
        m_mission_state = STATE_SURFACING;
        publishStates("OnNewMail_REQ_SURFACING_ACK_REC");
      }
    }
    else if ( key == "MISSION_TIME" )
    {
      std::cout << GetAppName() << " :: MISSION_TIME: " << sval
                << " at: " << currentMOOSTime() << std::endl;
      if ( sval == "end" && !m_final_hp_optim )
      {
        // end of adaptive mission, switch to final hp optimization
        // (if not in it already)
        m_final_hp_optim = true;

        if ( m_bhv_state != "hpoptim" )
          m_Comms.Notify("STAGE","hpoptim");

        if ( m_mission_state == STATE_SAMPLE ||
             m_mission_state == STATE_CALCWPT ||
             m_mission_state == STATE_CALCVOR )
        {
          // note:
          // if already in HPOPTIM state, we want to discard this optimization,
          // surface for data sharing, and then re-run hp optimization
          // stop any active HPOPTIM thread
          if ( m_hp_optim_running )
            m_cancel_hpo = true;

          if ( m_use_voronoi && !m_veh_is_shub )
            m_mission_state = STATE_REQ_SURF;
          else
          {
            m_last_surface = MOOSTime();
            m_mission_state = STATE_SURFACING;
          }
          publishStates("OnNewMail_MISSION_TIME");

          // reset other vars to make sure we can go over procedure again
          clearHandshakeVars();
          m_waiting = false;
          m_calc_prevoronoi = false;
          if ( m_debug )
            std::cout << GetAppName() << " :: setting m_calc_prevoronoi to false, MISSION_TIME msg" << std::endl;

        }
        else if ( m_mission_state != STATE_IDLE || m_mission_state != STATE_DONE )
        {
          // in all other cases
          if ( m_use_voronoi && !m_veh_is_shub )
            m_Comms.Notify("REQ_SURFACING","final");
        }

      }
    }
    else
      std::cout << GetAppName() << " :: Unhandled Mail: " << key << std::endl;

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
    // **** SAVING GP TO FILE (independent) **********************************//
    if ( m_mission_state != STATE_DONE && !(m_final_hp_optim) )
    {
      // periodically (every X s), store all GP predictions
      // if we did not do so in the last second (apptick)
      if ( (std::abs(m_last_pred_save - MOOSTime()) > 1.0) &&
           ((size_t)std::floor(currentMOOSTime()) % m_prediction_interval == 10) &&
           !m_final_hp_optim )
      {
        m_last_pred_save = MOOSTime();
        if ( m_debug )
          std::cout << GetAppName() << " :: updating m_last_pred_save to: "
                    << m_last_pred_save << std::endl;
        if ( m_verbose )
        {
          std::cout << GetAppName() << " :: creating thread to save state at: "
                    << currentMOOSTime() << std::endl;
        }
        std::thread pred_store(&GP::makeAndStorePredictions, this, false);
        pred_store.detach();
      }
    }

    // **** ACOMMS VORONOI PARTITIONING ************************************//
    if ( m_mission_state == STATE_SAMPLE && m_acomms_sharing &&
         m_use_voronoi && m_voronoi_subset.size() > 0 && !m_veh_is_shub )
    {
      #ifdef BUILD_VORONOI
      // check if we need to recalculate Voronoi region
      if ( needToRecalculateVoronoi() )
      {
        m_mission_state = STATE_CALCVOR;
        m_need_to_run_hpo = true;
        m_calc_prevoronoi = true;
        publishStates("Iterate_acomms_vor");
      }
      #endif
    }

    // **** HPOPTIM FOR 1 AUV **********************************************//
    if ( m_num_vehicles == 1 )
    {
      // run hpoptim every .. 500 seconds ..
      if ( (std::abs(m_last_hp_optim_done - MOOSTime()) > 1.0 ) &&
           ((size_t)std::floor(currentMOOSTime()) % 500 == 10) )
      {
        if ( m_mission_state != STATE_DONE )
          m_need_to_run_hpo = true;
      }
    }
    else
    {
      // start with a hyperparameter optimization
      if ( (MOOSTime() - m_start_time) < 1.0 ) //
      {
        if ( m_mission_state == STATE_IDLE )
          m_need_to_run_hpo = true;

        if ( m_veh_is_shub )
        {
          m_mission_state = STATE_SAMPLE;
          publishStates("Iterate_hpoptim_start shub");
        }
      }
    }

    // ***** RUN HP_OPTIM IN BACKGROUND ************************************//
    // for shub, check if done adding data,
    // if so, then kick off hpoptim
    if ( m_need_to_run_hpo && !m_hp_optim_running )
    {
      // && !m_first_surface
      if ( (m_veh_is_shub && m_queue_data_points_for_gp.size() <= 1) ||
           !m_veh_is_shub )
      {
        if ( m_verbose )
          std::cout << GetAppName() << " :: starting runHPOptimization, Iterate"
                    << std::endl;
        m_future_hp_optim = std::async(std::launch::async, &GP::runHPOptimization,
                                       this, m_hp_optim_iterations);
        m_hp_optim_running = true;
        if ( m_debug )
          std::cout << GetAppName() << " :: m_hp_optim_running = true, at: "
                    << currentMOOSTime() << std::endl;
        m_need_to_run_hpo = false;
      }
    }
    else if ( m_hp_optim_running )
    {
      try
      {
        if ( m_future_hp_optim.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
        {
          if ( m_verbose )
            std::cout << GetAppName() << " :: done running hp optim, at: "
                      << currentMOOSTime() << std::endl;
          m_hp_optim_running = false;
          if ( m_debug )
            std::cout << GetAppName() << " :: m_hp_optim_running = false, at: "
                      << currentMOOSTime() << std::endl;

          /*if ( !m_use_voronoi && !m_veh_is_shub )
          {
            clearTDSStateVars(); // TODO should we do this still? if hp in bg?
            if ( m_debug )
              std::cout << GetAppName() << " :: resetting from hp_optim done"
                        << std::endl;
          }
          else */
          if ( !m_veh_is_shub )
          { // use voronoi
            if ( m_mission_state == STATE_IDLE && !m_final_hp_optim )
            {
              // start of mission, no data yet, skip calcvor
              m_mission_state = STATE_SAMPLE;
              publishStates("Iterate_m_hp_optim_running");

              // init voronoi subset to whole region
              #ifdef BUILD_VORONOI
              if ( m_use_voronoi && !m_veh_is_shub )
              {
                std::cout << GetAppName() << " :: Initialize voronoi subset to whole region." << std::endl;
                // copy over all unvisited locations
                for ( auto map_item : m_sample_points_unvisited )
                  m_voronoi_subset.push_back(map_item.first);
                // calculate the convex hull
                voronoiConvexHull();
                // print convex hull
                printVoronoi();
              }
              #endif
            }
            else if ( m_mission_state == STATE_IDLE && m_final_hp_optim )
            {
              if ( m_verbose )
                std::cout << GetAppName() << " :: done with final HPO, at: "
                          << currentMOOSTime()
                          << " calling clearTDSStateVars()" << std::endl;
              clearTDSStateVars();
            }
            else
            {
              if ( m_verbose )
                std::cout << GetAppName()
                          << " :: starting calcMECriterion, m_future_calc_prevoronoi"
                          << std::endl;
              m_future_calc_prevoronoi = std::async(std::launch::async, &GP::calcMECriterion, this);
            }
          }
        }
      }
      catch ( const std::future_error& ex )
      {
        std::cout << GetAppName() << " :: future_error at Iterate hpoptim check shub: "
                  << ex.code() << ": " << ex.what()
                  << "at: " << currentMOOSTime() << std::endl;
        m_hp_optim_running = false;
        if ( m_debug )
          std::cout << GetAppName() << " :: m_hp_optim_running = false, at: "
                    << currentMOOSTime() << std::endl;
      }
    }

    // **** MAIN STATE MACHINE *********************************************//
    if ( m_debug )
      std::cout << GetAppName() << " :: Current state: " << currentMissionStateString() << std::endl;
    switch ( m_mission_state )
    {
      case STATE_SAMPLE :
        if ( m_timed_data_sharing && !m_use_voronoi && m_adp_state != "static" &&
             !m_veh_is_shub && m_num_vehicles > 1 )
        {
          if ( m_async_trigger_method == "timed" )
          {
            // TDS
            // let's time this based on MOOSTime(), and assume that clocks are
            // synchronised (to be verified in field tests)
            // note; add 60s buffer to block out x seconds at beginning
            if ( ((size_t)std::floor(currentMOOSTime()) % m_data_sharing_interval) == 0 &&
                 (size_t)std::floor(currentMOOSTime()) > 60 )
            {
              // switch to data sharing mode, to switch bhv to surface
              m_last_surface = MOOSTime();
              m_mission_state = STATE_SURFACING;
              publishStates("Iterate_STATE_SAMPLE_TDS");
            }
          }
          else if ( m_async_trigger_method == "num_other_samples" )
          {
            // trigger for async surfacing
            // calculate nr of other samples
            // sample frequency assumed to be 1 Hz
            // note that m_last_surface is set at the start of a surfacing event,
            //   and therefore we should subtract surfacing time to find how much
            //   data vehicles may have collected
            std::cout << GetAppName() << " :: MOOSTime(): " << MOOSTime()
                      << ", m_last_surface: " << m_last_surface
                      << ", m_twoway_time_to_surf: " << m_twoway_time_to_surf
                      << " combined: " << std::round(MOOSTime() - m_last_surface - m_twoway_time_to_surf)
                      << std::endl;
            int time_since_last_surf = std::round(MOOSTime() - m_last_surface - m_twoway_time_to_surf);
            if ( time_since_last_surf > 0 )
            {
              unsigned int other_samples = time_since_last_surf * (m_num_vehicles-1);
              double pct_total_time = currentMOOSTime() / m_mission_duration;
              double calc_time = 3*pow(10,-9)*pow(m_gp->get_sampleset_size()-2500, 3) + 10;
              if ( m_debug )
                std::cout << GetAppName() << " :: num_other_samples calc: "
                          << " time_since_last_surf: " << time_since_last_surf
                          << ", other_samples: " << other_samples
                          << ", m_time_to_surf: " << m_twoway_time_to_surf
                          << ", pct_total_time: " << pct_total_time
                          << ", calc_time: " << calc_time
                          << std::endl;

              // conditions: // TODO make sampling frequency explicit? assume now 1 Hz
              // 1. make sure we can gather more data than it costs to surface
              // 2. and decrease surfacing frequency with mission length (linear decrease?)
              // 3. don't surface too often, multiply by ? //TODO figure this out
              if ( other_samples >
                   ((1 + pct_total_time) * (m_twoway_time_to_surf + 5 + 1 + calc_time)) )
              { //   1 + _t_i / t_e          2*d_s              + d_b + d_e
                if ( m_debug )
                  std::cout << GetAppName() << " :: Time to Surface!" << std::endl;
                // switch to surfacing
                m_last_surface = MOOSTime();
                m_mission_state = STATE_SURFACING;
                publishStates("Iterate_num_other_samples");
              }
            }
          }
        }
        // tds with voronoi, trigger for when to request data sharing
        else if ( m_use_voronoi && m_voronoi_subset.size() > 0 &&
                  ((MOOSTime()-m_last_voronoi_calc_time) > m_vor_timeout) &&
                  m_adp_state != "static" && !m_veh_is_shub )
        {
          #ifdef BUILD_VORONOI
          if ( needToRecalculateVoronoi() )
          {
            // request surfacing through acomms
            m_mission_state = STATE_REQ_SURF;
            publishStates("Iterate_STATE_SAMPLE_need_to_recalc");
            clearHandshakeVars();
          }
          #endif
        }
        // else just sampling, don't do anything else

        break;
      case STATE_CALCWPT :
        findAndPublishNextWpt();
        break;
      case STATE_SURFACING :
        if ( m_bhv_state != "data_sharing" && !m_final_hp_optim )
          m_Comms.Notify("STAGE","data_sharing");

        if ( m_on_surface && m_num_vehicles > 1 )
        //(m_timed_data_sharing || m_use_voronoi) )
        {
          m_mission_state = STATE_HANDSHAKE;
          publishStates("Iterate_STATE_SURFACING_on_surface");
        }
        break;
      #ifdef BUILD_VORONOI
      case STATE_CALCVOR :
        if ( m_calc_prevoronoi && !m_hp_optim_running )
        {
          // check if done calculating the predictions, this is waiting
          // for both HP optimization and calcMECriterion
          if ( m_future_calc_prevoronoi.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
          {
            m_calc_prevoronoi = false;
          }
        }
        else if ( !m_calc_prevoronoi && !m_running_voronoi_routine )
        {
          m_running_voronoi_routine = true;
          runVoronoiRoutine();
          m_data_received = false; // TODO need this?
        }
        break;
      #endif
      case STATE_IDLE :
        break;
      // **** STEPS TO GET VEHICLES TO SURFACE *******************************//
      case STATE_REQ_SURF :
        // every second, generate var/val for acomms for surfacing
        if ( (MOOSTime() - m_last_published_req_surf) > 1.0 )
        {
          if ( !m_final_hp_optim )
            m_Comms.Notify("REQ_SURFACING","true");
          else
            m_Comms.Notify("REQ_SURFACING","final");
          m_last_published_req_surf = MOOSTime();
        }

        // timer to not wait too long
        m_req_surf_timer_counter++;
        if ( m_req_surf_timer_counter > (m_max_wait_for_other_vehicles * GetAppFreq()) )
        {
          // if we waited > X min, continue
          m_mission_state = STATE_SURFACING;
          std::cout << GetAppName() << " :: m_req_surf_timer_counter timeout @ "
                    << currentMOOSTime() << std::endl;
          publishStates("Iterate_STATE_REQ_SURF_timeout");
          m_req_surf_timer_counter = 0;
        }
        break;
      case STATE_ACK_SURF :
        // every second, generate var/val for acomms for surfacing
        if ( (MOOSTime() - m_last_published_req_surf_ack) > 1.0 )
        {
          if ( m_debug )
            std::cout << GetAppName() << " :: sending ACK" << std::endl;
          m_Comms.Notify("REQ_SURFACING_ACK","true");
          m_last_published_req_surf_ack = MOOSTime();
          m_nr_ack_sent++;
        }
        // when on surface, switch state to start handshake
        if ( m_on_surface && m_nr_ack_sent > 5 )
        {
          m_mission_state = STATE_HANDSHAKE;
          publishStates("Iterate_STATE_ACK_SURF_on_surface");
          // reset ack counter
          m_nr_ack_sent = 0;
        }
        break;
      case STATE_HANDSHAKE :
        tdsHandshake();
        break;
      case STATE_TX_DATA :
        // data already sent in tdsHandshake()
        if ( m_received_shared_data && !m_calc_prevoronoi )
        {
          // continue to receive data
          m_mission_state = STATE_RX_DATA;
          publishStates("Iterate_STATE_TX_DATA_received_data");
        }
        else if ( !m_calc_prevoronoi )
        {
          // wait for calculations to be done
          // run timer to avoid being stuck waiting for data
          m_tx_timer_counter++;
          // if waited for X min, continue
          if ( m_tx_timer_counter > (m_max_wait_for_other_vehicles*GetAppFreq()) )
          {
            m_received_shared_data = true;
            std::cout << GetAppName() << " :: m_tx_timer_counter timeout @ "
                      << currentMOOSTime() << std::endl;
            m_tx_timer_counter = 0;
          }
        }
        else
        {
          // shouldn't happen, printout for debug
          if ( m_debug )
            std::cout << GetAppName() << " :: stuck in TX_DATA, m_calc_prevoronoi:" << m_calc_prevoronoi << ", m_received_shared_data: " << m_received_shared_data << std::endl;
        }
        break;
      case STATE_RX_DATA :
        // 20170708 add in check to see if we received data yet, else we wait
        if ( !m_calc_prevoronoi && m_data_received )
          tdsReceiveData();
        else
        {
          // 20180326 put in fix to not hang on rx_data indefinitely (!m_data_received)
          // wait for data until timeout
          m_rx_timer_counter++;
          std::cout << GetAppName() << " :: STATE_RX_DATA waiting .. m_calc_prevoronoi: "
                    << m_calc_prevoronoi << ", m_data_received: " << m_data_received
                    << std::endl;
          if ( m_rx_timer_counter > (m_max_wait_for_other_vehicles*GetAppFreq()) )
          {
            std::cout << GetAppName() << " :: m_rx_timer_counter timeout @ "
                      << currentMOOSTime() << std::endl;
            m_rx_timer_counter = 0;

            // continue to wait for more data
            clearTDSStateVars();
          }
        }
      default :
        break;
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
    else if ( param == "nr_vehicles" )
    {
      m_num_vehicles = (size_t)atoi(value.c_str());
      if ( m_verbose )
        std::cout << GetAppName() << " :: nr_vehicles: " << m_num_vehicles << std::endl;
    }
    else if ( param == "output_var_share_data" )
      m_output_var_share_data = toupper(value);
    else if ( param == "input_var_share_data" )
      m_input_var_share_data = toupper(value);
    else if ( param == "timed_data_sharing" )
    {
      m_timed_data_sharing = (value == "true" ? true : false);

      if ( m_debug )
        std::cout << GetAppName() << " :: m_timed_data_sharing: " << m_timed_data_sharing << std::endl;
    }
    else if ( param == "data_sharing_interval" )
      m_data_sharing_interval = (size_t)atoi(value.c_str());
    else if ( param == "output_var_handshake_data_sharing" )
      m_output_var_handshake_data_sharing = toupper(value);
    else if ( param == "input_var_handshake_data_sharing" )
      m_input_var_handshake_data_sharing = toupper(value);
    else if ( param == "acomms_sharing" )
      m_acomms_sharing = (value == "true" ? true : false);
    else if ( param == "use_voronoi" )
      m_use_voronoi = (value == "true" ? true : false);
    else if ( param == "voronoi_timeout" )
      m_vor_timeout = (size_t)atoi(value.c_str());
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
    else if ( param == "hp_accepted_deviation_ratio" )
      m_hp_dev_ratio = (double)atof(value.c_str());
    else if ( param == "max_wait_for_other_vehicles" )
      m_max_wait_for_other_vehicles = (size_t)atoi(value.c_str());
    else if ( param == "surface_hub" )
      m_use_surface_hub = (value == "true" ? true : false);
    else if ( param == "async_trigger_method" )
      m_async_trigger_method = value;
    else if ( param == "async_threshold" )
      m_async_threshold = (double)atof(value.c_str());
    else if ( param == "survey_depth" )
      m_survey_depth = (double)atof(value.c_str());
    else if ( param == "survey_speed" )
      m_survey_speed = (double)atof(value.c_str());
    else if ( param == "dive_pitch_angle" )
      m_dive_pitch_angle = (double)atof(value.c_str());
    else if ( param == "mission_duration" )
      m_mission_duration = (double)atof(value.c_str());
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
  else
  {
    std::cout << GetAppName() << " :: vehicle name: " << m_veh_name << std::endl;
    if ( m_veh_name == "shub" )
      m_veh_is_shub = true;
  }

  registerVariables();

  std::thread data_thread(&GP::dataAddingThread, this);
  data_thread.detach();

  // init last calculations times to start of process
  m_start_time = MOOSTime();
  m_last_voronoi_calc_time = MOOSTime();
  m_last_published_req_surf = MOOSTime();
  m_last_published_req_surf_ack = MOOSTime();
  m_last_published = MOOSTime();
  m_last_surface = MOOSTime();

  // calculate once how long it approximately takes us to surface
  if ( m_survey_depth > 0 && m_dive_pitch_angle > 0 )
  {
    // calculate time to surface
    double surf_dist = m_survey_depth / sin(m_dive_pitch_angle * PI/180);
    m_twoway_time_to_surf = 2 * surf_dist / m_survey_speed;
  }

  if ( m_debug )
  {
    std::cout << GetAppName() << " :: AppTick: " << GetAppFreq() << std::endl;
    std::cout << GetAppName() << " :: m_prediction_interval: " << m_prediction_interval << std::endl;
    std::cout << GetAppName() << " :: m_use_voronoi: " << m_use_voronoi << std::endl;
    std::cout << GetAppName() << " :: m_timed_data_sharing: " << m_timed_data_sharing << std::endl;
    std::cout << GetAppName() << " :: m_data_sharing_interval: " << m_data_sharing_interval << std::endl;
    std::cout << GetAppName() << " :: m_acomms_sharing: " << m_acomms_sharing << std::endl;
    std::cout << GetAppName() << " :: m_use_surface_hub: " << m_use_surface_hub << std::endl;
    std::cout << GetAppName() << " :: m_async_trigger_method: " << m_async_trigger_method << std::endl;
    std::cout << GetAppName() << " :: m_async_threshold: " << m_async_threshold << std::endl;
    std::cout << GetAppName() << " :: m_mission_duration: " << m_mission_duration << std::endl;
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: initGeodesy
//            initialize MOOS Geodesy for lat/lon conversions
//
void GP::initGeodesy()
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

  // get current bhv state
  m_Comms.Register("STAGE", 0);
  m_Comms.Register("ADP_PTS", 0);

  // get trigger end mission
  m_Comms.Register("MISSION_TIME", 0);

  // get when wpt cycle finished
  // (when to run adaptive predictions in adaptive state)
  m_Comms.Register(m_input_var_adaptive_trigger, 0);

  // data sharing
  m_Comms.Register(m_input_var_share_data, 0);
  m_Comms.Register("LOITER_DIST_TO_POLY", 0);
  m_Comms.Register(m_input_var_handshake_data_sharing, 0);

  // get data from other vehicles
  m_Comms.Register("INCOMING_DATA_ACOMMS", 0);

  // get other vehicles' locations
  m_Comms.Register("NODE_REPORT", 0);

  // tmp test voronoi partitioning
  m_Comms.Register("TEST_VORONOI", 0);

  // surfacing requests
  m_Comms.Register("REQ_SURFACING_REC", 0); // receive surfacing request from other vehicle
  m_Comms.Register("REQ_SURFACING_ACK_REC", 0); // receive surfacing req ack from other vehicle

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
// Procedure: handleMailDataFromSensor
//            handle the incoming message
//
void GP::handleMailDataFromSensor(double received_data)
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
    if ( (m_use_log_gp && received_data > 0) || !m_use_log_gp )
    {
      // pass into data adding queue
      std::vector<double> nw_data_pt{veh_lon, veh_lat, received_data};
      m_queue_data_points_for_gp.push(nw_data_pt);
    }
    else
    {
      if ( m_verbose )
        std::cout << GetAppName() << " :: "
                  << "Error: using log_gp and received data <= 0: "
                  << received_data << std::endl;
    }

    // if we need to exchange data, then store for this purpose
    if ( m_num_vehicles > 1 )
      storeDataForSending(veh_lon, veh_lat, received_data);
  }
}

//---------------------------------------------------------
// Procedure: handleMailReceivedDataPts
//            taken the received data, and add them to the GP
//
size_t GP::handleMailReceivedDataPts(std::string incoming_data)
{
  // if this is the surface hub, we want to check who we got this data from
  // such that we can increment the index of data last sent, such that we
  // do not send the vehicle its own data back.
  std::string veh_nm;
  if ( m_use_surface_hub && m_veh_is_shub )
  {
    size_t index_colon = incoming_data.find_first_of(':');
    veh_nm = incoming_data.substr(0, index_colon);
    std::string remainder = incoming_data.substr(index_colon+1);
    size_t index_colon2 = remainder.find_first_of(':');
    std::string destination = remainder.substr(0, index_colon2);
    if ( m_debug )
      std::cout << GetAppName() << " :: received data from: " << veh_nm
                << " for: " << destination << std::endl;
    // extract actual data
    incoming_data = remainder.substr(index_colon2+1);
  }

  // parse string
  // 1. split all data points into a vector
  std::vector<std::string> sample_points = parseString(incoming_data, ';');
  size_t pts_added = sample_points.size();

  if ( m_verbose )
    std::cout << GetAppName() << " :: adding " << pts_added << " data points." << std::endl;
  if ( m_use_surface_hub && m_veh_is_shub)
  {
    // increase the index counter by the current number of pts_added
    std::map<std::string, size_t>::iterator veh_itr = m_map_vehicle_idx_data_last_sent.find(veh_nm);
    if ( veh_itr == m_map_vehicle_idx_data_last_sent.end() ) // nothing sent yet
    {
      // no index yet, create
      m_map_vehicle_idx_data_last_sent.insert(std::pair<std::string, size_t>(veh_nm, pts_added));
    }
    else
    {
      if ( m_debug )
        std::cout << GetAppName() << " :: Current index for " << veh_nm << " is: " << veh_itr->second << std::endl;
      // update the index - this assumes data is processed immediately and no data from another vehicle comes in between
      // the TX and RX state
      veh_itr->second = veh_itr->second + pts_added;
    }
    if ( m_debug )
      std::cout << GetAppName() << " :: New index for " << veh_nm << " is: " << veh_itr->second << std::endl;
  }

  // 2. for each, add to GP, use data adding thread, to reduce need for mutexes
  for ( std::string & data_pt_str : sample_points )
  {
    std::vector<std::string> data_pt_components = parseString(data_pt_str, ',');
    double veh_lon = atof(data_pt_components[0].c_str());
    double veh_lat = atof(data_pt_components[1].c_str());
    double data = atof(data_pt_components[2].c_str());

    // pass into data adding queue
    std::vector<double> nw_data_pt{veh_lon, veh_lat, data};
    m_queue_data_points_for_gp.push(nw_data_pt);
    // rest is handled by data adding thread
  }

  // wait until most data has been added
  if ( m_verbose )
    std::cout << GetAppName() << " :: current size queue data points: "
              << m_queue_data_points_for_gp.size() << std::endl;

  return pts_added;
}

//---------------------------------------------------------
// Procedure: handleMailSamplePoints
//            parse the string, store the sample locations
//
void GP::handleMailSamplePoints(std::string input_string)
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
void GP::handleMailSamplePointsSpecs(std::string input_string)
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
// Procedure: handleMailDataAcomms(std::string css)
//            take incoming comma-separated string (css)
//            parse, and if not from self, add data to GP
//
void GP::handleMailDataAcomms(std::string css)
{
  std::vector<std::string> incoming_str = parseString(css,',');
  std::string name = incoming_str[0].substr(5,incoming_str[0].length()-1);

  // do not do anything if this is my own message, or I am running HP optim
  // or I am finished
  if ( name == m_veh_name || m_hp_optim_running || m_finished )
    return;

  // if we immediately get the exact same data string, do not process
  if ( css == m_last_acomms_string )
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: received the same data through acomms, ignoring" << std::endl;
    return;
  }
  m_last_acomms_string = css;

  if ( m_verbose )
    std::cout << GetAppName() << " :: Received data from: " << name << std::endl;

  // for all data points in this string,
  // add to data adding queue
  for ( size_t cnt = 0; cnt < (incoming_str.size()-1)/4; cnt++ )
  {
    // grab data value for current i
    std::string d_str = incoming_str[cnt*4+4];
    double d_i = atof(d_str.substr(d_str.find('=')+1, d_str.length()).c_str());

    // if data is valid, process
    if ( d_i > 0 )
    {
      // grab x,y,z for current i
      std::string x_str = incoming_str[cnt*4+1];
      std::string y_str = incoming_str[cnt*4+2];
      std::string z_str = incoming_str[cnt*4+3];
      // get the values from the xi=XX string
      double x_i = atof(x_str.substr(x_str.find('=')+1, x_str.length()).c_str());
      double y_i = atof(y_str.substr(y_str.find('=')+1, y_str.length()).c_str());
      double z_i = atof(z_str.substr(z_str.find('=')+1, z_str.length()).c_str());

      // convert x,y to lon,lat
      double lon, lat;
      bool converted = convUTMToLonLat(x_i, y_i, lon, lat);
      // add to data adding queue
      if ( converted )
      {
        if ( m_verbose )
          std::cout << GetAppName() << " :: adding: " << lon << "," << lat << "," << d_i << std::endl;
        std::vector<double> nw_data_pt{lon, lat, d_i};
        m_queue_data_points_for_gp.push(nw_data_pt);
      }
    }
    else
      std::cout << GetAppName() << " :: received invalid data, ignoring" << std::endl;
  }

}

//---------------------------------------------------------
// Procedure: handleMailNodeReports(const std::string &input_string)
//            take incoming comma-separated string (css)
//            parse, and if not from self, store vehicle location
//
void GP::handleMailNodeReports(const std::string &input_string)
{
//  if ( m_debug )
//    std::cout << GetAppName() << " :: parsing: " << input_string << std::endl;
  // eg. NAME=anna,LAT=0,LON=10
  std::vector<std::string> str_tok = parseString(input_string, ',');

  // init vars for data we want
  double veh_lon, veh_lat;
  std::string veh_nm;

  // for each var=val pair, store only those we are interested in
  // quit if it is our own node_report
  for ( std::string const & varval : str_tok )
  {
    size_t equal_index = varval.find('=');
    if ( equal_index == std::string::npos )
    {
      std::cout << GetAppName() << " :: ERROR: cannot find '=' in string NODE_REPORT" << std::endl;
      return;
    }
    std::string var = varval.substr(0,equal_index);
    std::string val = varval.substr(equal_index+1,varval.length());

    if ( var == "NAME" && val == m_veh_name )
      return;

    if ( var == "NAME" )
      veh_nm = val;
    else if ( var == "LON" )
      veh_lon = atof(val.c_str());
    else if ( var == "LAT" )
      veh_lat = atof(val.c_str());
  }

  if ( veh_nm != "" && veh_nm != "shub" )
  { // only store if not surface hub
    // store the vehicle info
    if ( m_other_vehicles.find(veh_nm) == m_other_vehicles.end() )
        m_other_vehicles.insert(std::pair<std::string, std::pair<double, double> >(veh_nm,std::pair<double,double>(veh_lon, veh_lat)));
    else
      m_other_vehicles[veh_nm] = std::pair<double,double>(veh_lon,veh_lat);
  }
}


//---------------------------------------------------------
// Procedure: dataAddingThread()
//            continuously check the data point queue to see
//            if points need to be added to the GP, if so
//            then call func to add points to GP
//
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

      if ( m_debug )
        std::cout << GetAppName() << " :: adding data point: " << veh_lon
                  << "," << veh_lat << ":" << data_pt[2] << std::endl;
    }
  }
}

//---------------------------------------------------------
// Procedure: addPatternToGP(double veh_lon, double veh_lat, double value)
//            function to add points to GP, using mutex so as
//            to not disturb when other processes are reading from GP
//
void GP::addPatternToGP(double veh_lon, double veh_lat, double data_value)
{
  // limit scope mutex, protect when adding data
  // because this is now happening in a detached thread
  double location[2] = {veh_lon, veh_lat};

  // GP: pass in value
  // log GP: take log (ln) of measurement
  double save_val = m_use_log_gp ? log(data_value) : data_value;

  // downsampled data for hyperparam optimization
  if ( m_gp->get_sampleset_size() % m_downsample_factor == 0 )
  {
    std::vector<double> nw_data_pt{veh_lon, veh_lat, save_val};
    m_data_for_hp_optim.push(nw_data_pt);
  }

  // get the lock for GP
  std::unique_lock<std::mutex> ap_lock(m_gp_mutex, std::defer_lock);
  double lock_time1 = currentMOOSTime();
  while ( !ap_lock.try_lock() ){}
  double lock_time2 = currentMOOSTime();
  if ( lock_time2 - lock_time1 > 120.0 )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: ARGH: obtaining gp lock by addPatternToGP "
             << "took more than 120 seconds: " << (lock_time2 - lock_time1)
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }

  // Input vectors x must be provided as double[] and targets y as double.
  // add new data point to GP
  m_gp->add_pattern(location, save_val);
  // release mutex
  ap_lock.unlock();
//  if ( m_debug )
//    std::cout << GetAppName() << " :: gp lock released by addPatternToGP"
//              << ", at: " << currentMOOSTime() << std::endl;

  // update visited set if needed
  if ( !m_veh_is_shub )
  {
    int index = getIndexForMap(veh_lon, veh_lat);
    if ( index >= 0 )
      updateVisitedSet(veh_lon, veh_lat, (size_t)index);
  }
  // if this is the surface hub, we want to store received data points for
  // exchange to other vehicles
  else
    storeDataForSending(veh_lon, veh_lat, data_value);
}


//---------------------------------------------------------
// Procedure: storeDataForSending(double vlon, double vlat, double data)
//            for interval-based data sharing, push back data points that have
//            not been shared yet
//
void GP::storeDataForSending(double vlon, double vlat, double data)
{
  // save the data point in a vector that we will send
  std::ostringstream data_str_stream;
  data_str_stream << std::setprecision(10) << vlon << "," << vlat << ","
                  << std::setprecision(5) << data;

  if ( m_data_pt_counter > m_data_send_reserve || m_data_pt_counter == 0 )
  {
    // preallocate vector memory
    m_data_send_reserve += 1500;
    m_data_to_send.reserve(m_data_send_reserve);
  }

  m_data_to_send.push_back(data_str_stream.str());

  m_data_pt_counter++;
}

//---------------------------------------------------------
// Procedure: getIndexForMap
//            calculate the grid index for the vehicle location
//
int GP::getIndexForMap(double veh_lon, double veh_lat)
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
void GP::updateVisitedSet(double veh_lon, double veh_lat, size_t index )
{
  bool need_to_update_maps = false;

  // add mutex for changing of global maps
  std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);

  double lock_time1 = currentMOOSTime();
  while ( !map_lock.try_lock() ){}
  double lock_time2 = currentMOOSTime();
  if ( lock_time2 - lock_time1 > 120.0 )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: ARGH: obtaining map lock by updateVisitedSet "
             << "took more than 120 seconds: " << (lock_time2 - lock_time1)
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }

  std::unordered_map<size_t, Eigen::Vector2d>::iterator curr_loc_itr = m_sample_points_unvisited.find(index);
  if ( curr_loc_itr != m_sample_points_unvisited.end() )
    need_to_update_maps = true;

  if ( !need_to_update_maps )
  {
    map_lock.unlock();
//    if ( m_debug )
//    {
//      std::ostringstream cout_msg;
//      cout_msg << GetAppName() << " :: map lock released by: updateVisitedSet"
//                  << ", at: " << currentMOOSTime() << std::endl;
//      std::cout << cout_msg.str();
//    }
    return;
  }
  else
  {
    std::unordered_map<size_t, Eigen::Vector2d>::iterator curr_loc_itr = m_sample_points_unvisited.find(index);
    if ( curr_loc_itr != m_sample_points_unvisited.end() )
    {
      // remove point from unvisited set
      Eigen::Vector2d move_pt = curr_loc_itr->second;

      // check if the sampled point was nearby, if not, there's something wrong
      bool pt_nearby = checkDistanceToSampledPoint(veh_lon, veh_lat, move_pt);
      if ( !pt_nearby && m_mission_state == STATE_SAMPLE )
      {
        std::cout << GetAppName() << " :: ERROR? distance to sampled point is bigger than the grid spacing\n";
      }

      // add the point to the visited set
      m_sample_points_visited.insert(std::pair<size_t, Eigen::Vector2d>(index, move_pt));

      // and remove it from the unvisited set
      m_sample_points_unvisited.erase(curr_loc_itr);
      // if using voronoi region, remove from that set, if it is in there
      if ( m_use_voronoi && !m_veh_is_shub )
      {
        if ( m_voronoi_subset.size() > 0 )
          m_voronoi_subset.erase( std::remove(m_voronoi_subset.begin(), m_voronoi_subset.end(), index), m_voronoi_subset.end() );
        if ( m_voronoi_subset_other_vehicles.size() > 0 )
        {
          for ( auto veh : m_voronoi_subset_other_vehicles )
          {
            (veh.second).erase( std::remove((veh.second).begin(), (veh.second).end(), index), (veh.second).end() );
          }
        }
      }
//
//      // report
//      if ( m_verbose )
//      {
//        std::cout << '\n' << GetAppName() << " :: moved pt: " << std::setprecision(10) << move_pt(0) << ", " << move_pt(1);
//        std::cout << " from unvisited to visited.\n";
//        std::cout << GetAppName() << " :: Unvisited size: " << m_sample_points_unvisited.size() << '\n';
//        std::cout << GetAppName() << " :: Visited size: " << m_sample_points_visited.size() << '\n' << std::endl;
//      }
    }
  }
  map_lock.unlock();
//  if ( m_debug )
//  {
//    std::ostringstream cout_msg;
//    cout_msg << GetAppName() << " :: map lock released by: updateVisitedSet"
//                << ", at: " << currentMOOSTime() << std::endl;
//    std::cout << cout_msg.str();
//  }
}

//---------------------------------------------------------
// Procedure: checkDistanceToSampledPoint
//            we check if, after conversion, the distance
//            of sampled point to vehicle location is reasonable
//
bool GP::checkDistanceToSampledPoint(double veh_lon, double veh_lat, Eigen::Vector2d move_pt)
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
// Procedure: findNextSampleLocation
//            find next sample location, using mutual information
//
void GP::findNextSampleLocation()
{
  if ( m_verbose )
    std::cout << GetAppName() << " :: find nxt sample loc" << std::endl;

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
        m_future_next_pt = std::async(std::launch::async, &GP::calcMECriterion, this);
        m_finding_nxt = true;
      }
    }
    else
      std::cout << GetAppName() << " :: m_sample_points_unvisited is empty, unable to find next location" << std::endl;
  }
  else
  { // if ( m_voronoi_subset.size() == 0 )
    std::cout << GetAppName() << " :: GP is empty. Getting random location for initial sample location." << std::endl;

    #ifdef BUILD_VORONOI
    // if m_voronoi, init voronoi subset to whole region
    if ( m_use_voronoi && !m_veh_is_shub )
    {
      std::cout << GetAppName() << " :: Initialize voronoi subset to whole region." << std::endl;
      // copy over all unvisited locations
      for ( auto map_item : m_sample_points_unvisited )
        m_voronoi_subset.push_back(map_item.first);
      // calculate the convex hull
      voronoiConvexHull();
      // print convex hull
      printVoronoi();
    }
    #endif

    getRandomStartLocation();
  }
}

//---------------------------------------------------------
// Procedure: getRandomStartLocation
//            before there is data in the GP,
//            grab a random location to start from
//
void GP::getRandomStartLocation()
{
  int random_idx = (int)(rand() % (m_sample_locations.size()));
  std::pair<double, double> rand_loc = m_sample_locations.at(random_idx);

  std::ostringstream output_stream;
  output_stream << std::setprecision(15) << rand_loc.first << "," << rand_loc.second;
  m_Comms.Notify(m_output_var_pred, output_stream.str());

  // update state vars
  m_mission_state = STATE_SAMPLE;
  publishStates("getRandomStartLocation");
  m_last_published = MOOSTime();
}

//---------------------------------------------------------
// Procedure: greedyWptSelection()
//            check over all predictions to find best (greedy)
//
void GP::greedyWptSelection(Eigen::Vector2d & best_location)
{
// get next position, for now, greedy pick
  double best_so_far = -1*std::numeric_limits<double>::max();
  size_t best_so_far_idx = -1;

  // greedy: pick best
  if ( m_use_voronoi && !m_veh_is_shub )
  {
    if ( m_voronoi_subset.size() > 0 )
    {
      // check only for points in voronoi subset
      for ( auto loc : m_voronoi_subset )
      {
        std::unordered_map<size_t,double>::iterator  pt_pred_itr = m_unvisited_pred_metric.find(loc);
        if ( pt_pred_itr == m_unvisited_pred_metric.end() )
          std::cout << GetAppName() << " :: Error: could not find pt in prediction table" << std::endl;
        else
        {
          if ( pt_pred_itr->second > best_so_far)
          {
            best_so_far = pt_pred_itr->second;
            best_so_far_idx = loc;
          }
        }
      }
    }
    else
    {
      best_so_far_idx = -1;
      if ( m_verbose )
        std::cout << GetAppName() << " :: Error: no points in voronoi subset" << std::endl;
    }
  }
  else
  {
    // no voronoi regions, check for all unvisited locations
    for ( auto loc : m_unvisited_pred_metric )
    {
      if ( loc.second > best_so_far )
      {
        best_so_far = loc.second;
        best_so_far_idx = loc.first;
      }
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
      best_location = best_itr->second;

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
}


//---------------------------------------------------------
// Procedure: publishNextBestPosition
//            call Notify & publish location
//
void GP::publishNextBestPosition()
{
  // procedures for finding the next waypoint(s)
  Eigen::Vector2d next_wpt;
  greedyWptSelection(next_wpt);

  if ( next_wpt(0) == 0 && next_wpt(1) == 0 )
    std::cout << GetAppName() << " :: Error: no waypoint found yet." << std::endl;
  else
  {
    // publishing for behavior (pLonLatToWptUpdate)
    std::ostringstream output_stream;
    output_stream << std::setprecision(15) << next_wpt(0) << "," << next_wpt(1);
    m_Comms.Notify(m_output_var_pred, output_stream.str());

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
size_t GP::calcMECriterion()
{
  if ( m_verbose )
    std::cout << GetAppName() << " :: max entropy start" << std::endl;
  m_unvisited_pred_metric.clear();

  // stop calculations if we are surfacing suddenly
  if ( m_mission_state == STATE_SURFACING )
    return 0;

  std::time_t begin = std::time(0);
  double begin_moos_time = currentMOOSTime();

  if ( m_debug )
    std::cout << GetAppName() << " :: try for lock gp, calcMECriterion" << std::endl;
  // lock for access to m_gp
  std::unique_lock<std::mutex> gp_lock(m_gp_mutex, std::defer_lock);
  // use unique_lock here, such that we can release mutex after m_gp operation
  double lock_time1 = currentMOOSTime();
  while ( !gp_lock.try_lock() ){}
  double lock_time2 = currentMOOSTime();
  if ( lock_time2 - lock_time1 > 120.0 )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: ARGH: obtaining gp lock by calcMECriterion "
             << "took more than 120 seconds: " << (lock_time2 - lock_time1)
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }


  if ( m_debug )
    std::cout << GetAppName() << " :: make copy GP" << std::endl;
  // make a copy: reduce need for lock, but this does impact memory use
  libgp::GaussianProcess * gp_copy = new libgp::GaussianProcess(*m_gp);
  // release lock
  gp_lock.unlock();
  if ( m_debug )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: gp lock released by: calcMECriterion"
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }

  // stop calculations if we are surfacing suddenly
  if ( m_mission_state == STATE_SURFACING )
  {
    delete gp_copy;
    return 0;
  }

  if ( m_debug )
    std::cout << GetAppName() << " :: try for lock map" << std::endl;
  std::unique_lock<std::mutex> map_lock(m_sample_maps_mutex, std::defer_lock);
  lock_time1 = currentMOOSTime();
  while ( !map_lock.try_lock() ){}
  lock_time2 = currentMOOSTime();
  if ( lock_time2 - lock_time1 > 120.0 )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: ARGH: obtaining map lock by calcMECriterion "
             << "took more than 120 seconds: " << (lock_time2 - lock_time1)
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }

  // make copy of map to use instead of map,
  // such that we do not have to lock it for long
  std::unordered_map<size_t, Eigen::Vector2d> unvisited_map_copy;
  // calculate for all, because we need it for density voronoi calc for other vehicles
  unvisited_map_copy.insert(m_sample_points_unvisited.begin(), m_sample_points_unvisited.end());
  map_lock.unlock();
  if ( m_debug )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: map lock released by: calcMECriterion"
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }

  // stop calculations if we are surfacing suddenly
  if ( m_mission_state == STATE_SURFACING )
  {
    delete gp_copy;
    return 0;
  }

  if ( m_debug )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: calc max entropy"
              << ", size map: " << unvisited_map_copy.size()
              << ", size GP: " << gp_copy->get_sampleset_size()
              << ", size queue: " << m_queue_data_points_for_gp.size()
              << ", at: " << currentMOOSTime()
              << std::endl;
    std::cout << cout_msg.str();
  }


  double sum_var = 0;
  // for each unvisited location
  for ( auto y_itr : unvisited_map_copy )
  {
    // stop calculations if we are surfacing suddenly
    if ( m_mission_state == STATE_SURFACING )
      break;

    // get unvisited location
    Eigen::Vector2d y = y_itr.second;
    double y_loc[2] = {y(0), y(1)};

    // calculate its posterior entropy
    double pred_mean;
    double pred_cov;
    gp_copy->f_and_var(y_loc, pred_mean, pred_cov);

    sum_var += pred_cov;

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

    m_unvisited_pred_metric.insert(std::pair<size_t, double>(y_itr.first, post_entropy));
  }

  // for 'altruistic' surfacing trigger,
  // compare sum_var to sum_var at previous surfacing
  // and the threshold, to see whether or not to surface
  if ( m_async_trigger_method == "var_reduction" && m_timed_data_sharing &&
       !m_use_voronoi && !m_veh_is_shub )
  {
    if ( m_async_prev_sum_var_reset )
    {
      m_async_prev_sum_var = sum_var;
      if ( m_debug )
        std::cout << GetAppName() << " :: new sum_var: " << m_async_prev_sum_var << std::endl;
      m_async_prev_sum_var_reset = false;
    }
    else
    {
      // initial sum_var = 0, decrease = 1
      double decrease = 1.0 - (sum_var / m_async_prev_sum_var);
      if ( m_debug )
      {
        std::cout << GetAppName() << " :: var_decrease: "
                  << decrease << '\n';
        std::cout << GetAppName() << " :: m_calc_prevoronoi: "
                  << m_calc_prevoronoi << '\n';
        std::cout << GetAppName() << " :: current state: "
                  << currentMissionStateString() << std::endl;
      }

      if ( (std::abs(decrease) > m_async_threshold) && !m_calc_prevoronoi && !m_final_hp_optim )
      {
        if ( m_debug )
          std::cout << GetAppName() << " :: Time to Surface!"
                    << ", at: " << currentMOOSTime() << std::endl;
        // switch to surfacing
        m_mission_state = STATE_SURFACING;
        publishStates("calcMECriterion_Async_var_reduction");
        // update 'prev' var
        m_async_prev_sum_var = sum_var;
        if ( m_debug )
          std::cout << GetAppName() << " :: new sum_var: " << m_async_prev_sum_var << std::endl;
      }
    }
  }

  std::time_t end = std::time(0);
  double end_moos_time = currentMOOSTime();
  if ( m_verbose )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: Max Entropy calc time: "
              << std::difftime(end, begin)
              << " MOOSCalcTime: " << (end_moos_time-begin_moos_time)
              << " at: " << currentMOOSTime()
              << " MapSize: " << unvisited_map_copy.size()
              << " GPSize: " <<  gp_copy->get_sampleset_size()
              << std::endl;
    std::cout << cout_msg.str();
  }
  if ( m_async_trigger_method == "num_other_samples" &&
       m_calcwpt_from_data_sharing )
  {
    // set 'm_last_surface' to the end time of the surfacing event
    // to make sure we do not immediately surface again
    // (only if we came in CALCWPT from tdsReceiveData)
    m_last_surface = MOOSTime();
    m_calcwpt_from_data_sharing = false;
    if ( m_debug )
    {
      std::ostringstream cout_msg;
      cout_msg << GetAppName() << " :: reset m_last_surface, at: "
               << currentMOOSTime() << std::endl;
      std::cout << cout_msg.str();
    }
  }

  // copy of GP and unvisited_map get destroyed when this function exits
  delete gp_copy;
  return 0;
}


//---------------------------------------------------------
// Procedure: checkGPHasData
//            check that the GP has been filled, else quit
//
bool GP::checkGPHasData()
{
  return ( m_gp->get_sampleset_size() > 0 );
}


//---------------------------------------------------------
// Procedure: processReceivedData()
//            pass on received data points to be added to GP
//            via handleMailReceivedDataPts
//
size_t GP::processReceivedData()
{
  // avoid getting stuck at end, after extra send phase
  if ( m_final_hp_optim && m_final_received_from.size() == m_num_vehicles &&
       m_veh_is_shub && m_incoming_data_to_be_added.empty() )
    return 0;

  size_t pts_added = 0;
  unsigned int timer_counter =  0;
  while ( m_incoming_data_to_be_added.empty() &&
          timer_counter < (m_max_wait_for_other_vehicles*GetAppFreq())/2 )
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: waiting to receive data .. " << std::endl;

    // wait for calculations to be done
    // run timer to avoid being stuck waiting for data
    if ( (unsigned int)std::round(currentMOOSTime()) % 2 == 0 )
      timer_counter++;
  }
  // if waited for X min, continue
  if ( timer_counter > (m_max_wait_for_other_vehicles*GetAppFreq())/2 )
  {
    std::cout << GetAppName() << " :: RX_DATA timer_counter timeout @ "
              << currentMOOSTime() << std::endl;
    return 0;
  }

  if ( m_debug )
    std::cout << GetAppName() << " :: going to add "
              << m_incoming_data_to_be_added.size()
              << " sets of data points." << std::endl;
  while ( !m_incoming_data_to_be_added.empty() )
  {
    std::string data = m_incoming_data_to_be_added.back();
    m_incoming_data_to_be_added.pop_back();
    pts_added += handleMailReceivedDataPts(data);
  }
  return pts_added;
}

//---------------------------------------------------------
// Procedure: endMission()
//            set variables to end mission, initiate last save
//
void GP::endMission()
{
  // finish state machine
  m_mission_state = STATE_DONE;
  publishStates("endMission");

  // switch to final bhv
  m_Comms.Notify("STAGE","return");

  // store predictions after HP optim
  m_finished = true;

  if ( m_verbose )
  {
    std::cout << GetAppName()
              << " :: creating thread to save state at mission time (MOOSTime): "
              << currentMOOSTime() << ", finished." << std::endl;
  }
  std::thread pred_store(&GP::makeAndStorePredictions, this, true);
  pred_store.detach();
}


//---------------------------------------------------------
// Procedure: runHPOptimization(nr_iterations)
//            run in thread, call GP's hyperparam optimization
//
bool GP::runHPOptimization(size_t nr_iterations)
{
  // run hyperparameter optimization
  if ( m_verbose)
    std::cout << GetAppName() << " :: current size GP: " << m_gp->get_sampleset_size()
              << " at: " << currentMOOSTime() << std::endl;

  if ( m_cancel_hpo )
  {
    if ( m_verbose )
      std::cout << GetAppName() << " :: premature exit from HPO because of mission end, pre-optim, at "
                << currentMOOSTime() << std::endl;
    m_cancel_hpo = false;
    m_hp_optim_running = false;
    if ( m_debug )
      std::cout << GetAppName() << " :: m_hp_optim_running = false, at: "
                << currentMOOSTime() << std::endl;
    clearHandshakeVars();
    return false;
  }

  Eigen::VectorXd lh_gp(m_gp->covf().get_loghyper()); // via param function
  runHPoptimizationOnDownsampledGP(lh_gp, nr_iterations);

  if ( m_cancel_hpo )
  {
    if ( m_verbose )
      std::cout << GetAppName() << " :: premature exit from HPO because of "
                << "mission end, post-optim pre-set, at "
                << currentMOOSTime() << std::endl;
    m_cancel_hpo = false;
    m_hp_optim_running = false;
    if ( m_debug )
      std::cout << GetAppName() << " :: m_hp_optim_running = false, at: "
                << currentMOOSTime() << std::endl;
    clearHandshakeVars();
    return false;
  }

  // protect GP access with mutex
  std::unique_lock<std::mutex> hp_lock(m_gp_mutex, std::defer_lock);
  double lock_time1 = currentMOOSTime();
  while ( !hp_lock.try_lock() ){}
  double lock_time2 = currentMOOSTime();
  if ( lock_time2 - lock_time1 > 120.0 )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: ARGH: obtaining gp lock by runHPOptimization "
             << "took more than 120 seconds: " << (lock_time2 - lock_time1)
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }


  if ( m_cancel_hpo )
  {
    hp_lock.unlock();
    if ( m_verbose )
      std::cout << GetAppName() << " :: premature exit from HPO because of "
                << "mission end, pre-update lock (release gp lock), at "
                << currentMOOSTime() << std::endl;
    m_cancel_hpo = false;
    m_hp_optim_running = false;
    if ( m_debug )
      std::cout << GetAppName() << " :: m_hp_optim_running = false, at: "
                << currentMOOSTime() << std::endl;
    clearHandshakeVars();
    return false;
  }

  if ( m_verbose )
  {
    Eigen::VectorXd current_hps = m_gp->covf().get_loghyper();
    std::cout << GetAppName() << " :: m_GP hyper params before update: " << current_hps(0)
              << ", " << current_hps(1) << "," << current_hps(2) << std::endl;
  }

  // update the hyperparameters if the new ones are not too far removed
  // from the initial, to avoid messing up GP due to misestimations
  // most important factor is length scale
  //
  double length_scale = lh_gp(0);
  if ( m_debug )
    std::cout << GetAppName()
              << " :: ( std::abs((length_scale - m_prev_length_scale) / m_prev_length_scale) < "
              << m_hp_dev_ratio << " )? "
              << ( std::abs((length_scale - m_prev_length_scale) / m_prev_length_scale) < m_hp_dev_ratio )
              << " difference: " << std::abs((length_scale - m_prev_length_scale) / m_prev_length_scale)
              << '\n';

  if ( std::abs((length_scale - m_prev_length_scale) / m_prev_length_scale) < m_hp_dev_ratio )
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: updating hyper params!" << std::endl;
    m_gp->covf().set_loghyper(lh_gp);
    m_prev_length_scale = length_scale;
    // just update hyperparams. Call for f and var should init re-compute.
  }
  else
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: don't update hyper params! at "
                << currentMOOSTime() << std::endl;
  }

  //// write new HP to file ////////////////////////////////////////////////////
  std::time_t begin = std::time(0);
  std::stringstream filenm;
  filenm << "hp_optim_" << std::floor(currentMOOSTime()) << "_" << m_veh_name << "_" << nr_iterations;
  m_gp->write(filenm.str().c_str());
  std::time_t end = std::time(0);
  if ( m_debug )
    std::cout << GetAppName() << " :: HP param write to file time: "
              << std::difftime(end, begin) << '\n';

  hp_lock.unlock();
  if ( m_debug )
    std::cout << GetAppName() << " :: gp lock released by: runHPOptimization"
              << ", at: " << currentMOOSTime() << '\n';

  if ( m_verbose )
  {
    Eigen::VectorXd current_hps = m_gp->covf().get_loghyper();
    std::cout << GetAppName() << " :: new m_GP hyper params: " << current_hps(0)
              << ", " << current_hps(1) << "," << current_hps(2) << std::endl;
  }

  return true;
}

void GP::runHPoptimizationOnDownsampledGP(Eigen::VectorXd & loghp, size_t nr_iterations)
{
  //// downsample data for HP optimization /////////////////////////////////////
  std::time_t begin = std::time(0);
  double begin_moos_time = currentMOOSTime();

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
  std::time_t end = std::time(0);
  double end_moos_time = currentMOOSTime();
  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: runtime putting data into downsampled_gp: "
                              << std::difftime(end, begin)
                              << " MOOSTime: " << (end_moos_time-begin_moos_time)
                              << " at: " << currentMOOSTime() << std::endl;
    std::cout << GetAppName() << " :: size downsampled GP: "
                              << downsampled_gp.get_sampleset_size() << std::endl;
    std::cout << GetAppName() << " :: size orig GP: "
                              << m_gp->get_sampleset_size() << std::endl;
  }

  //// actual HP optimization /////////////////////////////////////
  begin = std::time(0);
  begin_moos_time = currentMOOSTime();
  // there are 2 methods in gplib, conjugate gradient and RProp,
  // the latter should be more efficient
  if ( m_hp_optim_cg )
  {
    libgp::CG cg;
    cg.maximize(&downsampled_gp, nr_iterations, 1);
  }
  else
  {
    libgp::RProp rprop;
    // RProp arguments: GP, 'n' (nr iterations), verbose
    rprop.maximize(&downsampled_gp, nr_iterations, 1);
  }
  end = std::time(0);
  end_moos_time = currentMOOSTime();
  if ( m_verbose )
    std::cout << GetAppName() << " :: runtime hyperparam optimization: "
              << std::difftime(end, begin)
              << " MOOSTime: " << (end_moos_time-begin_moos_time)
              << " at: " << currentMOOSTime() << std::endl;

  // downsampled gp should be destroyed upon exiting function
  loghp = downsampled_gp.covf().get_loghyper();

  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: downsampled GP hyper params: "
              << loghp(0) << ", " << loghp(1) << "," << loghp(2) << std::endl;
  }
}

//---------------------------------------------------------
// Procedure: sendReady()
//            notify other vehicles that this one is ready
//            for data exchange
//
void GP::sendReady()
{
  std::cout << GetAppName() << " :: send READY (" << m_output_var_handshake_data_sharing
            << ":" << m_veh_name << ")" << std::endl;
  Notify(m_output_var_handshake_data_sharing,m_veh_name);
}

//---------------------------------------------------------
// Procedure: sendData()
//            send data in chunks (via pShare)
//
void GP::sendData()
{
  if ( m_debug )
  {
    std::cout << GetAppName() << " :: entering sendData, at " << currentMOOSTime() << '\n';
    std::cout << GetAppName() << " :: m_rec_ready_veh.size(): " << m_rec_ready_veh.size() << '\n';

    std::cout << GetAppName() << " :: m_rec_ready_veh: ";
    for ( auto veh_idx = m_rec_ready_veh.begin(); veh_idx != m_rec_ready_veh.end(); ++veh_idx )
      std::cout << *veh_idx << ", ";
    std::cout << std::endl;
  }

  // there may be multiple vehicles waiting for data,
  // do this for all
  while ( !m_rec_ready_veh.empty() )
  {
    // grab one vehicle
    std::string received_ready_from = *m_rec_ready_veh.begin();
    m_rec_ready_veh.erase(received_ready_from);

    if ( m_final_hp_optim && m_veh_is_shub )
    {
      if ( m_debug )
        std::cout << GetAppName()
                  << " :: in sendData: m_final_received_from.size(), m_final_sent_to.size(): "
                  << m_final_received_from.size() << ", " << m_final_sent_to.size()
                  << std::endl;
      while ( std::find(m_final_sent_to.begin(), m_final_sent_to.end(), received_ready_from) != m_final_sent_to.end() )
      {
        if ( m_debug )
          std::cout << GetAppName() << " :: already sent data to: "
                    << received_ready_from << ", skipping." << std::endl;
        // data already sent to this vehicle, skip
        if ( m_rec_ready_veh.size() != 0 )
        { // get next one
          received_ready_from = *m_rec_ready_veh.begin();
          m_rec_ready_veh.erase(received_ready_from);
        }
        else
        {
          if ( m_verbose )
            std::cout << "No more vehicles are ready, exiting" << std::endl;
          return;
        }
      }
    }
    if ( m_debug )
      std::cout << GetAppName() << " :: sending data to: " << received_ready_from << std::endl;

    // know what data to send, index into vector
    size_t index_start, index_end, index_last_sent;

    // surface hub case, prepare then appropriate data
    size_t nr_stored_data_pts = m_data_to_send.size();
    index_end = nr_stored_data_pts;
    if ( m_debug )
      std::cout << GetAppName() << " :: nr_stored_data_pts: " << nr_stored_data_pts << std::endl;

    if ( m_veh_is_shub )
    {
      // for the surface hub, we need to know who is ready, to know what data to send
      if ( m_verbose )
        std::cout << GetAppName() << " :: sendData: received_ready_from: " << received_ready_from << std::endl;

      // check what we last sent to this vehicle, index into data vector
      std::map<std::string, size_t>::iterator veh_itr = m_map_vehicle_idx_data_last_sent.find(received_ready_from);
      if ( veh_itr == m_map_vehicle_idx_data_last_sent.end() ) // nothing sent yet
        index_last_sent = 0;
      else
        index_last_sent = veh_itr->second;

      // check how much data to send
      // if no new data is available yet, don't send anything
      // note; we assume we did not first receive data from the vehicle
      //       which, as per the state machine, should be an ok assumption
      if ( index_end <= index_last_sent )
        nr_stored_data_pts = 0; // force empty message
      else
        m_data_pt_counter = index_end - index_last_sent;

      // update the map item so 'last sent' refers to currently available nr data pts
      if ( veh_itr == m_map_vehicle_idx_data_last_sent.end() )
        m_map_vehicle_idx_data_last_sent.insert(std::pair<std::string, double>(received_ready_from, nr_stored_data_pts));
      else if ( index_end > index_last_sent )
        veh_itr->second = nr_stored_data_pts; // update only if changed since prev.

      if ( m_final_hp_optim &&
           std::find(m_final_sent_to.begin(), m_final_sent_to.end(), received_ready_from) == m_final_sent_to.end() )
      {
        m_final_sent_to.push_back(received_ready_from);
        if ( m_debug )
          std::cout << GetAppName() << " :: adding: " << received_ready_from << " to m_final_sent_to."
                    << " Size: " << m_final_sent_to.size() << ", at: " << currentMOOSTime() << std::endl;
      }
    }

    if ( m_data_pt_counter == 0 || nr_stored_data_pts == 0 )
    {
      // no data points to send, send empty msg
      std::string empty_msg = m_veh_name + ':' + received_ready_from + ':';
      m_Comms.Notify(m_output_var_share_data, empty_msg);
      if ( m_verbose )
      {
        std::ostringstream cout_msg;
        cout_msg << GetAppName() << " :: **no data stored, sending 0 points! "
                 << " (m_data_pt_counter, nr_stored_data_pts)" << m_data_pt_counter
                 << ", " << nr_stored_data_pts << std::endl;
        std::cout << cout_msg.str();
      }
    }
    else
    {
      // we need to chunk the data because pShare has a limit of 64K per message
      // 2000 points should be ca. 53K, so let's try that first
      // (should be ca. the amount to send at first HP optimization point)
      // note; 2000 seemed to stretch it, some complaint of 48kB,
      // let's do 1500, should be conservative enough
      size_t msg_cnt = 0;
      // limit to message size
      size_t nr_points_per_msg = 1500;

      if ( m_verbose )
      {
        std::ostringstream cout_msg;
        cout_msg << GetAppName() << " :: **sending " << m_data_pt_counter
                 << " points to " << received_ready_from
                 << ", size GP: " << m_gp->get_sampleset_size()
                 << ", size queue:" << m_queue_data_points_for_gp.size()
                 << ", at: " << currentMOOSTime()
                 << std::endl;

        std::cout << cout_msg.str();
      }

      while ( m_data_pt_counter != 0 )
      {
        // if less than 1500 pts remain, add all of those
        if ( m_data_pt_counter < 1500 )
          nr_points_per_msg = m_data_pt_counter;

        std::ostringstream data_string_stream;
        // add vehicle name (broadcast is also received by self)
        data_string_stream << m_veh_name << ":";
        data_string_stream << received_ready_from << ":";

        // send data in batches,
        // use msg_cnt to figure out indices (starts at 0)
        // and check if alltdsHandhake_received_ready has been sent via m_data_pt_counter
        index_start = msg_cnt*1500;

        // for the surface hub, start where we last left off
        if ( m_veh_is_shub )
          index_start = index_last_sent + index_start;

        index_end = index_start + nr_points_per_msg;

        // copy data into data_string_stream, semicolon separated
        std::copy(m_data_to_send.begin()+index_start, m_data_to_send.begin()+index_end, std::ostream_iterator<std::string>(data_string_stream,";"));

        // send MOOS message
        m_Comms.Notify(m_output_var_share_data,data_string_stream.str());

        // increase the nr messages sent (msg_cnt),
        // and reduce the nr of data points that still need to be send (m_data_pt_counter)
        msg_cnt++;
        m_data_pt_counter -= nr_points_per_msg;
      }
    }

    // remove data if this is not the surface hub
    // for surface hub, never clear/empty the m_data_to_send vector,
    //     because we keep indexing into this
    if ( !m_veh_is_shub )
    {
      // remove data from vector
      m_data_to_send.clear();
      // reset counter
      m_data_pt_counter = 0;
      // exit while loop
      break;
    }
  }
}

//---------------------------------------------------------
// Procedure: getLogGPPredMeanVarFromGPMeanVar
//            convert pred mean/var from GP to pred mean/var for log GP
//
void GP::getLogGPPredMeanVarFromGPMeanVar(double gp_mean, double gp_cov, double & lgp_mean, double & lgp_cov )
{
  // convert GP mean to log GP mean (lognormal mean)
  lgp_mean = exp(gp_mean + gp_cov/2.0);
  lgp_cov = (lgp_mean*lgp_mean) * (exp(gp_cov) - 1.0);
}

//---------------------------------------------------------
// Procedure: makeAndStorePredictions
//            file writing
//
void GP::makeAndStorePredictions(bool finished)
{
  std::time_t begin = std::time(0);
  double begin_moos_time = currentMOOSTime();
  // make a copy of the GP and use that below, to limit lock time
  std::unique_lock<std::mutex> gp_lock(m_gp_mutex, std::defer_lock);
  double lock_time1 = currentMOOSTime();
  while ( !gp_lock.try_lock() ){}
  double lock_time2 = currentMOOSTime();
  if ( lock_time2 - lock_time1 > 120.0 )
  {
    std::ostringstream cout_msg;
    cout_msg << GetAppName() << " :: ARGH: obtaining gp lock by makeAndStorePredictions "
             << "took more than 120 seconds: " << (lock_time2 - lock_time1)
             << ", at: " << currentMOOSTime() << std::endl;
    std::cout << cout_msg.str();
  }


  if ( m_verbose )
    std::cout << GetAppName() << " :: store predictions" << std::endl;
  // make a copy: reduce need for lock, but this does impact memory use
  libgp::GaussianProcess * gp_copy = new libgp::GaussianProcess(*m_gp);
  gp_lock.unlock();
  if ( m_debug )
    std::cout << GetAppName() << " :: gp lock released by: makeAndStorePredictions"
              << ", at: " << currentMOOSTime() << '\n';

  std::time_t end = std::time(0);
  double end_moos_time = currentMOOSTime();
  if ( m_verbose )
    std::cout << GetAppName() << " :: runtime mutex [makeAndStorePredictions]: "
              << std::difftime(end, begin)
              << " MOOSTime: " << (end_moos_time-begin_moos_time)
              << " at: " << std::floor(currentMOOSTime()) << std::endl;

  begin = std::time(0);
  begin_moos_time = currentMOOSTime();
  std::vector< std::pair<double, double> >::iterator loc_itr;
  // get the predictive mean and var values for all sample locations
  std::vector<double> all_pred_means_lGP;
  std::vector<double> all_pred_vars_lGP;
  std::vector<double> all_pred_mu_GP;
  std::vector<double> all_pred_sigma2_GP;
  // pre-alloc vectors
  size_t nr_sample_locations = m_sample_locations.size();
  all_pred_means_lGP.reserve(nr_sample_locations);
  all_pred_vars_lGP.reserve(nr_sample_locations);
  all_pred_mu_GP.reserve(nr_sample_locations);
  all_pred_sigma2_GP.reserve(nr_sample_locations);

  if ( m_debug )
    std::cout << GetAppName() << " :: size GP: "
              << gp_copy->get_sampleset_size()
              << ", at: " << currentMOOSTime() << std::endl;

  // make predictions for all sample locations
  for ( loc_itr = m_sample_locations.begin(); loc_itr < m_sample_locations.end(); loc_itr++ )
  {
    double loc[2] {loc_itr->first, loc_itr->second};
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
  end = std::time(0);
  end_moos_time = currentMOOSTime();
  if ( m_verbose )
    std::cout << GetAppName() << " :: runtime make predictions [makeAndStorePredictions]: "
              << std::difftime(end, begin)
              << " MOOSTime: " << (end_moos_time-begin_moos_time)
              << " at: " << std::floor(currentMOOSTime()) << std::endl;

  begin = std::time(0);
  begin_moos_time = currentMOOSTime();
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

  if ( finished )
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: " << currentMOOSTime() << " :: closing files." << std::endl;
    m_ofstream_pm_lGP.close();
    m_ofstream_pv_lGP.close();
    if ( m_use_log_gp )
    {
      m_ofstream_pmu_GP.close();
      m_ofstream_psigma2_GP.close();
    }
  }

  end = std::time(0);
  end_moos_time = currentMOOSTime();
  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: runtime save to file [makeAndStorePredictions]: "
              << std::difftime(end, begin)
              << " MOOSTime: " << (end_moos_time-begin_moos_time)
              << " at: " << std::floor(currentMOOSTime()) << std::endl;
  }

  // copy of GP gets destroyed when this function exits
  delete gp_copy;
}

//---------------------------------------------------------
// Procedure: calcLonLatSpacing
//            separate out these calculations
//            such that we only do them once
//
void GP::calcLonLatSpacing()
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
bool GP::convLonLatToUTM (double lon, double lat, double & lx, double & ly )
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
bool GP::convUTMToLonLat (double lx, double ly, double & lon, double & lat )
{
  bool successful = m_geodesy.UTM2LatLong(lx, ly, lat, lon);

  if ( !successful )
    std::cout << GetAppName() << " :: ERROR converting x/y to lon/lat.\n";

  return successful;
}

#ifdef BUILD_VORONOI
//---------------------------------------------------------
// Procedure: calcVoronoi
//            given all other vehicles (stored in map),
//            find the points within voronoi region,
//            over the unvisited set
//
void GP::calcVoronoi(double own_lon, double own_lat, std::map< std::string, std::pair<double,double> > other_centers )
{
  // clear out previous sets
  m_voronoi_subset.clear();
  m_voronoi_subset_other_vehicles.clear();

  if ( m_verbose )
    std::cout << GetAppName() << " :: own vehicle at: " << own_lon << "," << own_lat << std::endl;

  // if own vehicle is not in sample area,
  // we set Voronoi region to be whole area
  // this should only happen at the start of the adaptive sampling
  if ( !inSampleRectangle(own_lon, own_lat, true) )
  {
    if ( m_verbose )
      std::cout << GetAppName() << " :: vehicle not inside sample region, pushing whole region into voronoi subset" << std::endl;
    // copy over all unvisited locations
    for ( auto map_item : m_sample_points_unvisited )
      m_voronoi_subset.push_back(map_item.first);
  }
  else
  {
    // else, split sample area, given other vehicles
    // for all points in the unvisited set
    for ( auto pt : m_sample_points_unvisited )
    {
      size_t pt_key = pt.first;
      Eigen::Vector2d pt_loc = pt.second;
      // calculate distance to other vehicles,
      // and determine which is closest
      double min_dist = std::numeric_limits<double>::max();
      std::string closest_vehicle;
      if ( other_centers.size() > 0 )
      {
        for ( auto veh : other_centers )
        {
          double veh_lon = (veh.second).first;
          double veh_lat = (veh.second).second;

          if ( m_verbose )
          {
            // at first location, print out which vehicle we are checking for now
            if ( pt_key == m_sample_points_unvisited.begin()->first )
              std::cout << GetAppName() << " :: other vehicle: " << veh.first << " at: " << veh_lon << "," << veh_lat << '\n';
          }

          double dist_pt_to_veh = pow(pt_loc(0)-veh_lon,2) + pow(pt_loc(1)-veh_lat,2);
          if ( dist_pt_to_veh < min_dist )
          {
            min_dist = dist_pt_to_veh;
            closest_vehicle = veh.first;
          }
        }
      }
      // calculate distance to oneself
      double dist_to_self = pow(pt_loc(0)-own_lon, 2) + pow(pt_loc(1)-own_lat,2);
      if ( dist_to_self <= min_dist )
      {
        closest_vehicle = m_veh_name;
        // only in this case do we add the location to our Voronoi set
        m_voronoi_subset.push_back(pt_key);
      }
      else
      {
        auto other_veh = m_voronoi_subset_other_vehicles.find(closest_vehicle);
        if ( other_veh != m_voronoi_subset_other_vehicles.end() )
        {
          // vehicle already in subset map, add the point to its vector
          (other_veh->second).push_back(pt_key);
        }
        else
        {
          // vehicle not yet in subset map, insert here
          std::vector<size_t> nw;
          nw.push_back(pt_key);
          m_voronoi_subset_other_vehicles.insert(std::pair<std::string, std::vector<size_t>>(closest_vehicle, nw));
        }
      }
    } // for unvisited sample pts
  }
  if ( m_verbose )
  {
    std::cout << GetAppName() << " :: my set has: " << m_voronoi_subset.size() << " sample locations out of " << m_sample_points_unvisited.size() << std::endl;
    std::cout << GetAppName() << " :: other vehicle set sizes:\n";
    for ( auto veh : m_voronoi_subset_other_vehicles )
      std::cout << veh.first << ": " << (veh.second).size() << ", ";
    std::cout << std::endl;
  }


  if ( m_voronoi_subset.size() > 0 )
  {
    // calculate the convex hull
    voronoiConvexHull();

    // print convex hull
    printVoronoi();
  }
}
#endif

//---------------------------------------------------------
// Procedure: inSampleRectangle(double veh_lon, double veh_lat, bool use_buffer) const
//            see if the current vehicle position is inside the
//            prespecified sampling area (rectangle)
//
bool GP::inSampleRectangle(double veh_lon, double veh_lat, bool use_buffer) const
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

#ifdef BUILD_VORONOI
//---------------------------------------------------------
// Procedure: voronoiConvexHull()
//            get the convex hull of the Voronoi region
//
void GP::voronoiConvexHull()
{
  // collect points inside voronoi region in boost geometry multi_point
  boost::geometry::clear(m_voronoi_pts);

  for ( size_t vor_pt : m_voronoi_subset )
  {
    //auto vor_loc = vor_pt.second;
    auto vor_loc = m_sample_points_unvisited.find(vor_pt)->second;
    double pt_lon = vor_loc(0);
    double pt_lat = vor_loc(1);
    boost::geometry::append(m_voronoi_pts, boost_pt(pt_lon, pt_lat));
  }

  // next, get the convex hull for these points
  m_voronoi_conv_hull.clear();
  boost::geometry::convex_hull(m_voronoi_pts, m_voronoi_conv_hull);

  if ( m_verbose )
    std::cout << GetAppName() << " :: convex hull pts: " << (size_t)boost::geometry::num_points(m_voronoi_conv_hull) << std::endl;
}

//---------------------------------------------------------
// Procedure: inVoronoi(double lon, double lat) const
//            check if given location is inside convex hull of Voronoi region
//
bool GP::inVoronoi(double lon, double lat) const
{
  boost_pt pt_to_check(lon, lat);
  return boost::geometry::intersects(pt_to_check, m_voronoi_conv_hull);
}

//---------------------------------------------------------
// Procedure: distToVoronoi(double lon, double lat) const
//            calculate distance vehicle location to boundaries voronoi convex hull
//
double GP::distToVoronoi(double lon, double lat) const
{
  boost_pt pt_to_check(lon, lat);

  // iterate over vertices in convex hull to create lines to calc distance to
  double min_dist = std::numeric_limits<double>::max();
  auto bst_ext_ring = boost::geometry::exterior_ring(m_voronoi_conv_hull);
  for ( auto itr = boost::begin(bst_ext_ring); itr != boost::end(bst_ext_ring); ++itr )
  {
    // line between current and next point
    boost::geometry::model::linestring<boost_pt> line;
    if ( itr+1 != boost::end(bst_ext_ring) )
    {
      boost_pt bpt = *itr;
      boost_pt nxt = *(itr+1);
      boost::geometry::append(line, bpt);
      boost::geometry::append(line, nxt);
    }
    // calculate distance to line
    if ( boost::geometry::num_points(line) > 0 )
    {
      double curr_dist = boost::geometry::distance(pt_to_check, line);

      // store if less than current min
      if ( curr_dist < min_dist )
        min_dist = curr_dist;
    }
  }

  return min_dist;
}

//---------------------------------------------------------
// Procedure: printVoronoiConvexHull()
//            print voronoi convex hull vertices to cout and MOOS
//
void GP::printVoronoi()
{
  auto bst_ext_ring = boost::geometry::exterior_ring(m_voronoi_conv_hull);
  std::ostringstream voronoi_str;

  if ( m_verbose )
    std::cout << GetAppName() << " :: voronoi convex hull: " << boost::geometry::dsv(bst_ext_ring) << std::endl;

  for ( auto itr = boost::begin(bst_ext_ring); itr != boost::end(bst_ext_ring); ++itr )
  {
    boost_pt bpt = *itr;
    voronoi_str << bpt.get<0>() << "," << bpt.get<1>() << ";";
  }

  m_Comms.Notify("VORONOI_REGION",voronoi_str.str());

  if ( m_verbose )
    printVoronoiPartitions();
}
#endif


//---------------------------------------------------------
// Procedure: tdsHandshake
//            broadcast that vehicle is ready when on surface
//            and share data when received 'ready' from other vehicle
//            assumes only 2 vehicles
//
void GP::tdsHandshake()
{
  size_t moos_t = (size_t)std::floor(MOOSTime());

  if ( m_debug )
    std::cout << GetAppName() << " :: tdsHandhake -- mission_state: " << currentMissionStateString() << std::endl;

  if ( !m_on_surface )
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: not on surface yet, do not yet send handshake" << std::endl;
  }
  else
  {
    // send data
    if ( m_received_ready )
    {
      // other vehicle already ready to exchange data
      // send that we are ready, when we are at the surface
        sendReady();

      // send the data
      m_last_ready_sent = moos_t;
      //m_data_sharing_state = DATA_SEND;
      m_mission_state = STATE_TX_DATA;
      publishStates("tdsHandhake_received_ready");

      sendData();

      // reset for next time
      m_received_ready = false;
      // reset handshake timer in case of data received while it was already counting
      m_handshake_timer_counter = 0;
    }
    else
    {
      // this vehicle is ready to exchange data, other vehicle not yet,
      // keep sending that we are ready:
      // every 2 seconds, notify that we are ready for data exchange
      if ( moos_t % 2 == 0 &&
           moos_t - m_last_ready_sent > 1 )
      {
        sendReady();
        m_last_ready_sent = moos_t;
      }

      // update timer for timeout
      m_handshake_timer_counter++;
      // if we have not received all ready messages within X minutes, proceed
      if ( m_handshake_timer_counter > (m_max_wait_for_other_vehicles * GetAppFreq())
          && !m_veh_is_shub )
      {
        if ( (m_use_surface_hub && !m_final_hp_optim) || !m_use_surface_hub )
        m_received_ready = true;
        std::cout << GetAppName() << " :: m_handshake_timer_counter timeout @ "
                  << currentMOOSTime() << std::endl;
        m_handshake_timer_counter = 0;
      }
    }
  }
}

//---------------------------------------------------------
// Procedure: tdsReceiveData
//            start processing received data, and when all
//            are process, then switch state to continue with
//            the survey mission
//
void GP::tdsReceiveData()
{
  if ( !m_waiting )
  {
    // data received, need to add
    m_future_received_data_processed = std::async(std::launch::async, &GP::processReceivedData, this);
    if ( m_debug )
      std::cout << GetAppName() << " :: kicking off processReceivedData" << std::endl;
    m_waiting = true;
  }
  else if ( m_waiting && !m_calc_prevoronoi )
  {
    if ( m_debug && (size_t)std::floor(currentMOOSTime()) % 2 == 0 )
      std::cout << GetAppName() << " :: checking future m_future_received_data_processed" << std::endl;

    try {
      if ( m_future_received_data_processed.wait_for(std::chrono::milliseconds(1)) ==
           std::future_status::ready )
      {
        size_t pts_added = m_future_received_data_processed.get();

        if ( m_verbose )
          std::cout << GetAppName() << " ::  added: " << pts_added << " data points" << std::endl;

        // for the var_reduction approach, we want to not trigger based on
        // information gathered by other vehicle, so set a flag here such that
        // in calcMECriterion we know not to trigger
        if ( m_async_trigger_method == "var_reduction" )
          m_async_prev_sum_var_reset = true;

        // run HP optimization
        if ( m_first_surface )
          m_first_surface = false;

        // for the surface hub, be ready to send/receive more data,
        // and run hp optimization in the background.
        if ( !m_veh_is_shub )
        {
          m_need_to_run_hpo = true;
          if ( m_final_hp_optim )
          {
            // skip voronoi calculation on final data exchange
            m_mission_state = STATE_IDLE;
            publishStates("tdsReceiveData_vor_final");
            // wait for HPO to be done,
            // which calls clearTDSStateVars and ends mission
          }
          else if ( !m_use_voronoi )
          {
            m_mission_state = STATE_CALCWPT;
            publishStates("tdsReceiveData");
            m_calcwpt_from_data_sharing = true;
          }
          else
          { // m_use_voronoi
            m_calc_prevoronoi = true; // wait for HP optim and calcMECriterion
            m_mission_state = STATE_CALCVOR;
            publishStates("tdsReceiveData_vor_not_final");
          }
        }
        else
        { // shub
          if ( m_debug )
          {
            std::cout << GetAppName() << " :: shub, be ready to send/receive data, "
                      << "processing data in the background" << '\n';
            std::cout << GetAppName() << " :: m_final_received_from.size(), m_final_sent_to.size(): "
                      << m_final_received_from.size() << ", " << m_final_sent_to.size() << std::endl;
          }

          // Nb. check if data processed, and kick off hpoptim in Iterate
          m_need_to_run_hpo = true;

          // continue to wait for more data
          clearTDSStateVars();
        }
      }
      // else, continue waiting
    }
    catch (const std::future_error& ex)
    {
      std::cout << GetAppName() << " :: Caught a future_error with code: "
                << ex.code() << " and message: " << ex.what()
                << "in tdsReceiveData() checking m_future_received_data_processed."
                << std::endl;
    }
  }
}


//---------------------------------------------------------
// Procedure: clearTDSStateVars
//            reset state vars for TDS control
//
void GP::clearTDSStateVars()
{
  if ( m_verbose )
    std::cout << GetAppName() << " :: reset state vars" << std::endl;

  // move to next step; need wpt
  if ( m_adaptive && m_num_vehicles > 1 && !m_final_hp_optim &&
       m_mission_state != STATE_CALCWPT )
  {
    std::cout << GetAppName() << " :: STATE_CALCWPT via clearTDSStateVars" << std::endl;
    m_mission_state = STATE_CALCWPT;
    publishStates("clearTDSStateVars_adp_nrveh_gt_1");
  }
  else if ( !m_final_hp_optim &&
            m_mission_state != STATE_SAMPLE &&
            m_mission_state != STATE_CALCWPT )
  {
    m_mission_state = STATE_SAMPLE;
    publishStates("clearTDSStateVars_else");
  }
  else if ( m_final_hp_optim && !m_veh_is_shub )
    endMission();

  if ( m_bhv_state != "survey" &&
       ((m_veh_is_shub && !m_final_hp_optim) || !m_veh_is_shub ) )
    m_Comms.Notify("STAGE","survey");

  // reset surfacing/handshake vars
  m_waiting = false;
  if ( m_use_voronoi && !m_veh_is_shub )
  {
    // move to next step: voronoi calc & predictions done
    m_precalc_pred_voronoi_done = true;
  }

  if ( m_timed_data_sharing || m_use_voronoi )
    clearHandshakeVars();

  // for shub, at end, wait to receive data from both vehicles before running hpoptim
  // this should be the only place where there is a transition RX -> HS
  //
  // also, to avoid the case where things get stuck because
  // one of the vehicles is already in RX_DATA,
  // (because READY is undirected and can advance HS)
  // switch to STATE_HANDSHAKE
  //
  if ( m_veh_is_shub )
  {
    m_mission_state = STATE_HANDSHAKE;
    publishStates("clearTDSStateVars shub");
    if ( m_final_received_from.size() == m_num_vehicles && m_final_sent_to.size() < m_num_vehicles )
      m_received_ready = true;
    else
    {
      if ( m_final_received_from.size() == m_num_vehicles &&
           m_final_sent_to.size() == m_num_vehicles )
      {
        // ending
        endMission(); // check if this is the proper way to end
      }
      else
      {
        if ( m_debug )
          std::cout << GetAppName() << " :: m_final_received_from.size(), m_final_sent_to.size(): "
                    << m_final_received_from.size() << ", " << m_final_sent_to.size()
                    << std::endl;
      }
    }
  }
}

//---------------------------------------------------------
// Procedure: clearHandshakeVars
//            clear handshake counters
//
void GP::clearHandshakeVars()
{
  // reset surfacing/handshake other vehicles sets
  if ( !m_veh_is_shub && !m_final_received_from.size() > 0 )
  {
    // we clear the vector here in case 'ready' is received again while
    // we are sending the data, which seems to happen, but I removed it before
    // to avoid problems at the end for the shub, so adding that condition
    m_rec_ready_veh.clear();
  }

  m_rec_ack_veh.clear();
  if ( m_debug )
    std::cout << GetAppName() << " :: m_rec_ack_veh, m_rec_shared_data, m_rec_ready"
              << " reset in clearHandshakeVars, at: " << currentMOOSTime()
              << std::endl;

  m_received_shared_data = false;
  m_received_ready = false;
}

//---------------------------------------------------------
// Procedure: findAndPublishNextWpt
//            calculate the next sample location,
//            and publish to MOOSDB when found
//
void GP::findAndPublishNextWpt()
{
  // only run calculation of predictions if we did not already do
  // this during the voronoi calculation
  if ( m_precalc_pred_voronoi_done )
  {
    // publish best location
    publishNextBestPosition();

    // reset vars
    m_precalc_pred_voronoi_done = false;
    m_finding_nxt = false;
  }
  else
  {
    if ( !m_finding_nxt )
    {
      if ( m_verbose )
        std::cout << GetAppName() << " :: calling to find next sample location" << std::endl;
      findNextSampleLocation();
    }
    else
    {
      try {
        // see if we can get result from future
        std::cout << GetAppName() << " :: checking future m_future_next_pt" << std::endl;
        if ( m_future_next_pt.wait_for(std::chrono::microseconds(1)) == std::future_status::ready )
        {
          m_finding_nxt = false;

          // publish greedy best
          publishNextBestPosition();

          // continue survey
          if ( m_bhv_state != "survey" )
            m_Comms.Notify("STAGE","survey");
        }
        else
        {
//          if ( m_debug )
//          {
//            std::cout << GetAppName() << " :: m_timed_data_sharing, !m_use_voronoi, m_veh_name: "
//                      << m_timed_data_sharing << ", " << m_use_voronoi << ", " << m_veh_name << std::endl;
//          }
          if ( m_timed_data_sharing && !m_use_voronoi && !m_veh_is_shub )
          {
            // for TDS, we need to make sure we initiate surfacing,
            // even if we were finding a new waypoint,
            // such that we keep all vehicles synched
            if ( m_num_vehicles > 1 &&
                 ((size_t)std::floor(currentMOOSTime()) % m_data_sharing_interval) == 0 &&
                 (size_t)std::floor(currentMOOSTime()) > 60 &&
                 m_async_trigger_method == "timed" )
            {
              // switch to data sharing mode, to switch bhv to surface
              m_mission_state = STATE_SURFACING;
              publishStates("findAndPublishNextWpt");
            }
          }
        } // else future check
      }
      catch ( const std::future_error& ex )
      {
        std::cout << GetAppName() << " :: Caught a future_error with code: " << ex.code()
                  << " and message: " << ex.what() << "in findAndPublishNextWpt() checking m_future_next_pt." << std::endl;
      }
    } // else !m_finding_nxt
  } // else m_precalc_pred_voronoi_done
}

#ifdef BUILD_VORONOI
//---------------------------------------------------------
// Procedure: needToRecalculateVoronoi
//            see if we need to recalculate the voronoi region
//            for now, when we are close to voronoi border
//
bool GP::needToRecalculateVoronoi()
{
  // recalculate if within ca. half lat/lon spacing removed fromborder
  // of the voronoi region
  double voronoi_threshold = ((m_lon_spacing + m_lat_spacing)/2.0) / 2.0;
  double dist_to_voronoi = distToVoronoi(m_lon, m_lat);
  m_Comms.Notify("DIST_TO_VORONOI", dist_to_voronoi);
  if ( dist_to_voronoi < voronoi_threshold )
    return true;
  return false;
}
#endif

//---------------------------------------------------------
// Procedure: ownMessage(std::string input)
//            check if this message is from own vehicle
//
bool GP::ownMessage(std::string input)
{
  size_t index = input.find(m_veh_name);
  return ( index != std::string::npos );
}

//---------------------------------------------------------
// Procedure: finalSurface(std::string input)
//            check if message contains word 'final'
//
bool GP::finalSurface(std::string input)
{
  // note; we give value 'final', but surface request is a bool
  // given message size, so it would be converted to 'false'
  size_t index = input.find("false");
  return ( index != std::string::npos );
}

#ifdef BUILD_VORONOI
//---------------------------------------------------------
// Procedure: runVoronoiRoutine()
//            after calculation predictions, calculate
//            and recalculate voronoi regions
//
void GP::runVoronoiRoutine()
{
  if ( m_debug )
    std::cout << GetAppName() << " :: calcVoronoi requested by runVoronoiRoutine (1)."
              << "at: " << currentMOOSTime() << std::endl;
  calcVoronoi(m_lon, m_lat, m_other_vehicles);

  // got initial voronoi partitioning
  // now let's use density function to get new voronoi initiators
  // k-means like, we do this until some convergence
  //
  double own_centroid_lon(0.0), own_centroid_lat(0.0);
  std::map<std::string, std::pair<double, double> > other_vehicle_centroids;

  double prev_own_centroid_lon(0.0), prev_own_centroid_lat(0.0);
  std::map<std::string, std::pair<double, double> > prev_other_vehicle_centroids;

  std::cout << GetAppName() << " :: run calcVoronoiCentroids till convergence! \n";
  size_t conv_ctr = 0;
  while ( !centroidConvergence(prev_own_centroid_lon, prev_own_centroid_lat, prev_other_vehicle_centroids,
                                 own_centroid_lon, own_centroid_lat, other_vehicle_centroids) )
  {
    std::cout << GetAppName() << " :: previously (lon, lat, other): " << own_centroid_lon << ", "
              << own_centroid_lat << ", ";
    for ( auto other_veh : other_vehicle_centroids )
      std::cout << (other_veh.second).first << ", " << (other_veh.second).second << "; ";
    std::cout << '\n';

    // save previous locations
    if ( own_centroid_lon == 0.0 )
      prev_own_centroid_lon = m_lon;
    else
      prev_own_centroid_lon = own_centroid_lon;
    if ( own_centroid_lat == 0.0 )
      prev_own_centroid_lat = m_lat;
    else
      prev_own_centroid_lat = own_centroid_lat;
    prev_other_vehicle_centroids.clear();
    if ( other_vehicle_centroids.size() == 0 )
      prev_other_vehicle_centroids.insert(m_other_vehicles.begin(), m_other_vehicles.end());
    else
      prev_other_vehicle_centroids.insert(other_vehicle_centroids.begin(), other_vehicle_centroids.end());

    // calculate new locations
    calcVoronoiCentroids(own_centroid_lon, own_centroid_lat, other_vehicle_centroids );

    if ( m_verbose )
    {
      std::cout << GetAppName() << " :: old centroids: " << prev_own_centroid_lon << "," << prev_own_centroid_lat << ";";
      for ( auto veh : prev_other_vehicle_centroids )
        std::cout << (veh.second).first << "," << (veh.second).second << ";";
      std::cout << '\n';
      std::cout << GetAppName() << " :: new centroids: " << own_centroid_lon << "," << own_centroid_lat << ";";
      for ( auto veh : other_vehicle_centroids )
        std::cout << (veh.second).first << "," << (veh.second).second << ";";
      std::cout << '\n';
    }

    // now, recalculate the voronoi partitioning, given the new centroids
    // if it is not zero
    if ( std::abs(own_centroid_lon - own_centroid_lat) > 1 )
    {
      if ( m_debug )
        std::cout << GetAppName() << " :: calcVoronoi requested by runVoronoiRoutine (2),"
                  << " at: " << currentMOOSTime() << '\n';
      calcVoronoi(own_centroid_lon, own_centroid_lat, other_vehicle_centroids);
    }

    conv_ctr++;
  }

  std::cout << GetAppName() << " :: nr convergence iterations: " << conv_ctr << '\n';

  m_last_voronoi_calc_time = MOOSTime();

  std::cout << GetAppName() << " :: clearTDSStateVars via runVoronoiRoutine" << std::endl;
  clearTDSStateVars();

  m_calc_prevoronoi = false;
  m_running_voronoi_routine = false;
}


bool GP::centroidConvergence ( double old_lon, double old_lat, std::map<std::string, std::pair<double, double> > old_centr,
                               double new_lon, double new_lat, std::map<std::string, std::pair<double, double> > new_centr )
{
  // degrees = meters / {lat,long}_deg_to_m
  double threshold = 1.0 / (m_lon_deg_to_m + m_lat_deg_to_m / 2.0) ;

  if ( old_centr.empty() )
  {
    std::cout << GetAppName() << " :: old_centr is empty, no convergence yet."
              << std::endl;
  }
  else if ( old_centr.size() != new_centr.size() )
  {
    std::cout << GetAppName() << " :: ERROR, incorrect map sizes." << std::endl;
    std::cout << GetAppName() << " :: old_centr_size, new_centr_size: "
              << old_centr.size() << ", " << new_centr.size() << std::endl;
  }
  else
  {
    bool significant_diff = false;
    if ( sqrt( pow(old_lon-new_lon, 2) + pow(old_lat - new_lat, 2) ) > threshold )
      significant_diff = true;
    for ( std::map<std::string, std::pair<double, double> >::iterator itr = old_centr.begin(); itr != old_centr.end(); ++itr )
    {
      std::pair<double, double> old_loc = itr->second;
      std::map<std::string, std::pair<double, double> >::iterator nw_itr = new_centr.find(itr->first);
      std::pair<double, double> new_loc = nw_itr->second;
      double diff = sqrt( pow(old_loc.first - new_loc.first,2) + pow(old_loc.second - new_loc.second,2) );
      std::cout << GetAppName() << " :: test convergence, threshold: " << threshold << " vs. diff: " << diff << std::endl;
      if ( diff > threshold )
        significant_diff = true;
    }
    // if no significant difference for any centroid, then there is convergence
    std::cout << GetAppName() << " :: sign diff? " << significant_diff << std::endl;
    return !significant_diff;
  }
  return false;
}


//---------------------------------------------------------
// Procedure: calcVoronoiCentroids()
//            calculate the centroids of each voronoi
//            partition, given the density function
//
void GP::calcVoronoiCentroids(double & own_centroid_lon, double & own_centroid_lat, std::map< std::string, std::pair<double,double> > & other_vehicle_centroids )
{
  // voronoi partitions are in: m_voronoi_subset (vector) and
  //    m_voronoi_subset_other_vehicles (map)
  // density function: values output by metric for unsampled
  //    locations are stored in m_unvisited_pred_metric

  // own
  own_centroid_lon = 0.0;
  own_centroid_lat = 0.0;
  if ( m_debug )
    std::cout << GetAppName() << " :: calculate own\n";
  if ( m_voronoi_subset.size() > 0 )
    calcVoronoiPartitionCentroid(m_voronoi_subset, own_centroid_lon, own_centroid_lat);
  else
  {
    own_centroid_lon = m_lon;
    own_centroid_lat = m_lat;
  }

  // others
  if ( m_voronoi_subset_other_vehicles.size() == 0 )
  {
    if ( m_debug )
      std::cout << GetAppName() << " :: m_voronoi_subset_other_vehicles is empty" << std::endl;
    return;
  }
    std::cout << GetAppName() << " :: calculate others" << std::endl;
  if ( m_debug )
  for ( auto veh : m_voronoi_subset_other_vehicles )
  {
    double centr_lon(0.0), centr_lat(0.0);
    std::vector<size_t> veh_voronoi_subset = veh.second;
    if ( veh_voronoi_subset.size() > 0 )
      calcVoronoiPartitionCentroid(veh_voronoi_subset, centr_lon, centr_lat);

    auto veh_map_itr = other_vehicle_centroids.find(veh.first);
    if ( veh_map_itr == other_vehicle_centroids.end() )
      other_vehicle_centroids.insert(std::pair<std::string, std::pair<double, double> >(veh.first, std::pair<double, double>(centr_lon, centr_lat)));
    else
      veh_map_itr->second = std::pair<double, double>(centr_lon, centr_lat);
  }

  // check if there are vehicles for whom there is not
  // yet a voronoi subset,
  // if so, then set their centroids to vehicle position
  for ( auto veh : m_other_vehicles )
  {
    auto itr = other_vehicle_centroids.find(veh.first);
    if ( itr == other_vehicle_centroids.end() )
    {
      if ( m_verbose )
        std::cout << GetAppName() << " :: no centroid for " << veh.first << ", adding vehicle position." << std::endl;
      // add vehicle position as centroid
      other_vehicle_centroids.insert(veh);
    }
  }
}

//---------------------------------------------------------
// Procedure: calcVoronoiPartitionCentroid( std::vector<size_t> voronoi_partition, double & centroid_lon, double & centroid_lat )
//            calculate weighted centroid of voronoi partition
//
void GP::calcVoronoiPartitionCentroid( std::vector<size_t> voronoi_partition, double & centroid_lon, double & centroid_lat )
{
  // grab the metric values for each point
  double sum_wt = 0.0;
  double sum_val_lon = 0.0;
  double sum_val_lat = 0.0;

  double min_wt = std::numeric_limits<double>::max();
  for ( auto prediction : m_unvisited_pred_metric )
  {
    if ( prediction.second < min_wt )
      min_wt = prediction.second;
  }

  for ( auto idx : voronoi_partition )
  {
    // get density func / metric value for current location (pt in voronoi region)
    std::unordered_map<size_t, double>::iterator pred_itr = m_unvisited_pred_metric.find(idx);
    double wt;
    if ( pred_itr == m_unvisited_pred_metric.end() )
    {
      // item from voronoi_partition not found in m_unvisited_pred_metric
      // note; should we change it to calc metric for all locations (not just unvisited?)
      std::cout << GetAppName() << " :: Prediction not found, mismatch "
                << "m_unvisited_pred_metric and voronoi_partition, add zero."
                << std::endl;
      wt = 0.0;
    }
    else
    {
      // shift weights to be > 0, if necessary
      wt = (min_wt < 0) ? std::abs(min_wt) + pred_itr->second : pred_itr->second;
    }
    if ( m_debug )
    {
      if ( wt < 0 )
        std::cout << GetAppName() << " :: weight smaller than zero: " << wt << std::endl;
    }

    // get current location (pt in voronoi region)
    // and store weighted location
    std::unordered_map<size_t, Eigen::Vector2d>::iterator pt_itr = m_sample_points_unvisited.find(idx);
    if ( pt_itr == m_sample_points_unvisited.end() )
      std::cout << GetAppName() << " :: Error: voronoi pt not in unvisited set?" << std::endl;
    else
    {
      Eigen::Vector2d loc = pt_itr->second;
      sum_wt += wt;
      sum_val_lon += wt*loc(0);
      sum_val_lat += wt*loc(1);
    }
  }
  if ( m_debug )
    std::cout << GetAppName() << " :: sum_wt: " << sum_wt << std::endl;

  // then calculate the centroid, given density
  centroid_lon = (sum_wt > 0 ? (sum_val_lon / sum_wt) : -1.0);
  centroid_lat = (sum_wt > 0 ? (sum_val_lat / sum_wt) : -1.0);
}

void GP::printVoronoiPartitions()
{
  std::cout << GetAppName() << " :: own partition has: " << m_voronoi_subset.size() << " points\n";
  for ( auto veh : m_voronoi_subset_other_vehicles )
    std::cout << GetAppName() << " :: partition for " << veh.first << " has " << (veh.second).size() << " points\n";

  std::cout << std::endl;
}
#endif

void GP::publishStates(std::string const calling_method)
{
  if ( m_verbose )
    std::cout << GetAppName() << " :: ** " << currentMOOSTime() << " switch to: " << currentMissionStateString() << " from " << calling_method << " **\n";

  m_Comms.Notify("STATE_MISSION", currentMissionStateString());
}

double GP::currentMOOSTime() const
{
  return MOOSTime() - m_start_time;
}
