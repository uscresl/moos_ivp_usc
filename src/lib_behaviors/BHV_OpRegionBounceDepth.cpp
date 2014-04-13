/*****************************************************************/
/*    Name: Stephanie Kemna (SK) <kemna@nurc.nato.int>           */
/*    Organization: NATO Undersea Research Centre, La Spezia, IT */
/*    File: BHV_OpRegionBounceDepth.cpp                          */
/*    Date: May 11th, 2011                                       */
/*          Taken out of BHV_OpRegionBounce,                     */
/*          an adaptation of BHV_OpRegion                        */
/*                                                               */
/*    Copyright (C) 2011  NURC                                   */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program.                              */
/* If not, see <http://www.gnu.org/licenses/                     */
/*                                                               */
/*****************************************************************/
/*                                                               */
/* BHV_OpRegionBounceDepth:                                      */
/*  Instead of setting speed to 0 when we hit the depth/altitude */
/*  limits, we want the vehicle to bounce back: change depth     */
/*                                                               */
/*****************************************************************/

#ifdef _WIN32
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif

#include <iostream>
#include <math.h> 
#include <stdlib.h>

#include "BHV_OpRegionBounceDepth.h"
#include "MBUtils.h"

#include "ZAIC_PEAK.h"  // obviously, to get the ZAIC_PEAK
#include "BuildUtils.h" // for subDomain

//-----------------------------------------------------------
// Procedure: Constructor
//
BHV_OpRegionBounceDepth::BHV_OpRegionBounceDepth(IvPDomain gdomain) : IvPBehavior(gdomain)
{
  this->setParam("descriptor", "bhv_opregionbouncedepth");
  m_domain = subDomain(m_domain, "depth");

  // default values in case not set
  m_max_depth = 0;
  m_min_altitude = 0;

  // IvP params as behaviour params
  m_peakwidthDepth = 4.0;
  m_basewidthDepth = 4.0;
  m_summitdelta = 40.0;
  m_minutil = 0.0; // static
  m_maxutil = 100.0; // static

  // initialize buffers to 0 in case parameters are not set
  m_depth_pwt = 0;
  m_depth_buffer = 0;
  m_no_zone_factor = 0.2;

  // debugging output?
  m_debug = false;
}

//-----------------------------------------------------------
// Procedure: setParam
//     Notes: We expect the "waypoint" entries will be of the form
//            "xposition,yposition,tag" where the tag is an optional
//            descriptor of the waypoint.
//            The "radius" parameter indicates what it means to have
//            arrived at the waypoint.
//
bool BHV_OpRegionBounceDepth::setParam(std::string param, std::string val) 
{
  if(IvPBehavior::setParam(param, val))
    return(true);

  if(param == "max_depth") {
    double dval = atof(val.c_str());
    if((dval < 0) || (!isNumber(val)))
      return(false);
    m_max_depth = dval;
    return(true);
  }
  else if(param == "min_altitude") {
    double dval = atof(val.c_str());
    if((dval < 0) || (!isNumber(val)))
      return(false);
    m_min_altitude = dval;
    return(true);
  }
  // 2011-04 SK: size of the depth buffer for bouncing
  else if(param == "bounce_buffer_depth") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_depth_buffer = dval;
    return(true);
  }
  // 2011-04 SK: no zone factor, no_zone = buffer * no_zone_factor
  else if(param == "no_zone_factor") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_no_zone_factor = dval;
    return(true);
  }  
  // 2011-03 SK: IvP params as parameters, possible to set, I did not 
  // include these in the plug, because you should only change them if 
  // you know what you're doing. The standard values are pretty 
  // reasonable for most purposes.
  else if(param == "peakwidth_depth") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_peakwidthDepth = dval;
    return(true);
  }
  else if(param == "basewidth_depth") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_basewidthDepth = dval;
    return(true);
  }
  else if(param == "summitdelta") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_summitdelta = dval;
    return(true);
  }
  // SK: Nb. minutil is always 0, maxutil calculated locally, max 100.0
  return(false);
}


//-----------------------------------------------------------
// Procedure: onRunState
//     Notes: Always returns NULL, never returns an IvPFunction*
//     Notes: Sets state_ok = false and posts an error message if
//            any of the OpRegionBounce conditions are not met.
//
IvPFunction *BHV_OpRegionBounceDepth::onRunState() 
{
  // initialize IvP functions to avoid crashing the helm
  m_ipfDepth = 0;
  // reset previous depths and weights
  unsigned int i;
  for(i=0; i < 2; i++) {
    m_safe_depth[i] = 0.0;
    m_safe_depth_weights[i] = 0.0;
  }
  // reset to no emergencies.
  m_depth_emergency = false;
   
  IvPFunction *ipf = 0; // SK: initialize to avoid crashing the helm

  depthVerify();
  altitudeVerify();

  if(m_depth_emergency)
  {
    createDepthBounce();
    ipf = m_ipfDepth;
    ipf->setPWT(m_depth_pwt);
  }

  if(ipf) {
    if(m_debug) {
      std::cout << std::endl << "** Created Depth IPF **" << std::endl;
      ipf->getPDMap()->getDomain().print();
      ipf->getPDMap()->print();
      std::cout << "PWT : " << ipf->getPWT();
      std::cout << std::endl;
    }
    return(ipf);
  }
  else {
    if(m_debug)
      std::cout << "** No Depth IPF Created **" << std::endl;
    return NULL;
  }
}

//-----------------------------------------------------------
// Procedure: depthVerify()
//      Note: Verify that the depth limit hasn't been violated.
//    Return: void, writes to class vars
//
void BHV_OpRegionBounceDepth::depthVerify()
{
  // If no max_depth specified, return with no error message posted.
  if(m_max_depth <= 0)
    return;

  bool ok;
  double depth = getBufferDoubleVal("NAV_DEPTH", ok);
  // SK: no_zone: highest utility zone
  double no_zone = m_depth_buffer*m_no_zone_factor;

  // Must get ownship depth from info_buffer
  if(!ok) { 
    //cout << "No NAV_DEPTH in info_buffer for vehicle: " << m_us_name << endl;
    postEMessage("No ownship depth in info_buffer.");
    //m_info_buffer->print();
    return;
  }

  // 2011-04 SK: set params for depth buffer bounce before error.
  if(ok && (m_depth_buffer > 0) && (depth > (m_max_depth-m_depth_buffer))) {
    m_depth_emergency = true;

    // new depth: max - buffer - a bit extra
    m_safe_depth[0] = m_max_depth-m_depth_buffer-no_zone;
    // weight: maxutil*(1-(max-a bit extra)-depth)/(buffer-a bit extra)
    m_safe_depth_weights[0] = m_maxutil*(1.0-(((m_max_depth-no_zone)-depth)/m_depth_buffer));
    if(m_safe_depth_weights[0] > m_maxutil)
      m_safe_depth_weights[0] = m_maxutil;
  }

  // just bounce, depth error handled by BHV_OpRegionBounce
}

//-----------------------------------------------------------
// Procedure: altitudeVerify()
//      Note: Verify that the altitude limit hasn't been violated.
//    Return: void, writes to class vars
//
void BHV_OpRegionBounceDepth::altitudeVerify()
{
  // If no min_altitude specified, return with no error message posted.
  if(m_min_altitude <= 0)
    return;

  bool ok, ok2;
  double curr_altitude = getBufferDoubleVal("NAV_ALTITUDE", ok);
  double depth = getBufferDoubleVal("NAV_DEPTH", ok2); // SK need current depth for calc safe depth
  double no_zone = m_depth_buffer*m_no_zone_factor;

  // Must get ownship altitude from info_buffer
  if(!ok) { 
    postEMessage("No ownship altitude in info_buffer.");
    return;
  }

  // 2011-04 SK: set params for altitude buffer bounce before error.
  if((m_depth_buffer > 0.0) && (curr_altitude < (m_min_altitude+m_depth_buffer)) && ok && ok2 && (curr_altitude > 0.0)) {
    // bounce the vehicle back up
    m_depth_emergency = true;
    // new altitude: min + buffer + a bit extra
    double wanted_altitude = m_min_altitude + m_depth_buffer + no_zone;
    // because of emergency, current altitude < wanted, therefore subtract
    double diff_altitude = wanted_altitude - curr_altitude;
    // subtract this difference from the current depth to bounce up
    m_safe_depth[1] = depth - diff_altitude;
    m_safe_depth_weights[1] = m_maxutil*(1.0-((curr_altitude - (m_min_altitude+no_zone))/m_depth_buffer));
    if(m_safe_depth_weights[1] > m_maxutil)
      m_safe_depth_weights[1] = m_maxutil;
  }

  // just bounce, altitude error handled by BHV_OpRegionBounce
}

//-----------------------------------------------------------
// Procedure: createDepthBounce()
// SK 2011-04
//   Purpose: To construct the depth objective function in case of
//            a bounce
//   Return:  void (saves to class var)
//
void BHV_OpRegionBounceDepth::createDepthBounce()
{  
  if(m_debug)
    std::cout << std::endl << "** BHV_OpRegionBounce::createDepthBounce" << std::endl;
    
  // there can only be two possible depth issues: 
  // through depthVerify() and altitudeVerify()
  ZAIC_PEAK zaicDepth(m_domain, "depth");
  // ZAIC Params: summit, peakwidth, basewidth, summitdelta, minutil, maxutil, index=0 

  // if both are present, then scale smaller utility, set highest to 100
  if((m_safe_depth_weights[0] > 0.0) && (m_safe_depth_weights[1] > 0.0)) {
    if(m_safe_depth_weights[0] > m_safe_depth_weights[1]) {
      // depth calculated by depthVerify of higher importance
      zaicDepth.setParams(m_safe_depth[0], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, m_maxutil);
      unsigned int index = zaicDepth.addComponent();
      double factor = m_maxutil/m_safe_depth_weights[0];
      double maxutil =  m_safe_depth_weights[1]*factor;
      zaicDepth.setParams(m_safe_depth[1], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, maxutil, index);
    }
    else if(m_safe_depth_weights[1] > m_safe_depth_weights[0]) {
      // depth calculated by altitudeVerify of higher importance
      double factor = m_maxutil/m_safe_depth_weights[1];
      double maxutil =  m_safe_depth_weights[0]*factor;
      zaicDepth.setParams(m_safe_depth[0], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, maxutil);
      unsigned int index = zaicDepth.addComponent();
      zaicDepth.setParams(m_safe_depth[1], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, m_maxutil, index);
    }
    else {
      // same util, create both with m_maxutil
      zaicDepth.setParams(m_safe_depth[0], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, m_maxutil);
      unsigned int index = zaicDepth.addComponent();
      zaicDepth.setParams(m_safe_depth[1], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, m_maxutil, index);
    }
  }
  else if(m_safe_depth_weights[0] > 0.0) {
    // just depth failure
    zaicDepth.setParams(m_safe_depth[0], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, m_maxutil);
  }
  else if(m_safe_depth_weights[1] > 0.0) {
    // just altitude failure
    zaicDepth.setParams(m_safe_depth[1], m_peakwidthDepth, m_basewidthDepth, m_summitdelta, m_minutil, m_maxutil);
  }

  // scale the calculated utilities (range [0-100]) if the behaviour's
  // pwt was higher than standard (100, = m_maxutil)
  if(m_priority_wt > 100.0) {
    m_safe_depth_weights[0] *= (m_priority_wt/100.0);
    m_safe_depth_weights[1] *= (m_priority_wt/100.0);
  }
  // determine behaviour weight: take highest individual weight.
  // The above code will have properly rescaled the other weight if 
  // present, so that the final function has the right relation between
  // zaic components.
  if(m_safe_depth_weights[0] >= m_safe_depth_weights[1])
    m_depth_pwt = m_safe_depth_weights[0];
  else
    m_depth_pwt = m_safe_depth_weights[1];

  // debug output
  if(m_debug) {
    std::string zaic_warnings = zaicDepth.getWarnings();
    if(zaic_warnings != "") 
      postWMessage(zaic_warnings);

    std::cout << " Depth IPF: " << std::endl;
    std::cout << "  peakwidth: " << m_peakwidthDepth << std::endl;
    std::cout << "  basewidth: " << m_basewidthDepth << std::endl;
    std::cout << "  m_summitdelta: " << m_summitdelta << std::endl; 
    std::cout << "  m_minutil: " << m_minutil << std::endl;
    std::cout << "  m_safe_depth[0] " << m_safe_depth[0] << " @  " << m_safe_depth_weights[0] << std::endl;
    std::cout << "  m_safe_depth[1] " << m_safe_depth[1] << " @  " << m_safe_depth_weights[1] << std::endl;
  }

  // save function
  zaicDepth.setSummitInsist(true);
  // extractIvPFunction(false): combine using sum of values <- wanted
  //                   (true) : take the max value of all
  m_ipfDepth = zaicDepth.extractIvPFunction(false);

  if(m_debug && m_ipfDepth)
    m_ipfDepth->getPDMap()->print();

  if(m_debug)
    std::cout << "** BHV_OpRegionBounce::createDepthBounce finished" << std::endl;  
}
