/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: BHV_OpRegionBounce.cpp                               */
/*    DATE: May 1st, 2005 Sunday at Joe's in Maine               */
/*                                                               */
/*    Adapted to bounce, both bounce on polygon and for depth    */
/*    Name: Stephanie Kemna (SK) <kemna@nurc.nato.int>           */
/*    Organization: NATO Undersea Research Centre, La Spezia, IT */
/*    Started: CASW11 Engineering Trial, March 2011, Ligurian Sea*/
/*    nb. I tried to follow programming style of the document,   */
/*        and included comments where I added stuff              */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*                                                               */
/*****************************************************************/
/*                                                               */
/* BHV_OpRegionBounce:                                           */
/*  Instead of setting speed to 0 when we hit the OpRegion or    */
/*  altitude limits, we want the vehicle to bounce back into     */
/*  the OpArea: change heading (and/or depth)                    */
/*                                                               */
/*****************************************************************/

#ifdef _WIN32
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif
#include <iostream>
#include <math.h> 
#include <stdlib.h>
#include "BHV_OpRegionBounce.h"
#include "MBUtils.h"
#include "XYFormatUtilsPoly.h"
// SK: added headers
#include "GeomUtils.h"  // for distPointToSeg
#include "AngleUtils.h" // for mod360
#include "ZAIC_PEAK.h"  // obviously, to get the ZAIC_PEAK
#include "BuildUtils.h" // for subDomain
#include "OF_Coupler.h" // for the OF_Coupler (be able to use 2 domains)

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_OpRegionBounce::BHV_OpRegionBounce(IvPDomain gdomain) : IvPBehavior(gdomain)
{
  this->setParam("descriptor", "bhv_opregionbounce");

  // 2011-05-11 SK: Coupler seems to not be working properly,
  //                putting depth in separate bhv
  m_couple = false;
  if(m_couple)
    m_domain = subDomain(m_domain, "course,speed,depth");
  else
    m_domain = subDomain(m_domain, "course,speed");  

  m_max_depth    = 0;
  m_min_altitude = 0;
  m_max_time     = 0;

  // Keep track of whether the vehicle was in the polygon on the
  // previous invocation of the behavior. Initially assume false.
  m_previously_in_poly = false;

  // Keep a flag indicating whether this is the first time the
  // behavior is invoked.
  m_first_time   = true;

  // Time stamps for calculating how long the vehicle has been
  // inside or out of the polygon.
  m_previous_time = 0;
  m_current_time  = 0;
  m_elapsed_time  = 0;
  m_start_time    = 0;
  m_delta_time    = 0;
  m_secs_in_poly  = 0;
  m_secs_out_poly = 0;
  
  // Declare whether the polygon containment condition is effective
  // immediately (default) or triggered only when the vehicle has
  // first entered the polygon region. This is useful if the vehicle
  // is being launched from a dock or area which is outside the 
  // safe zone
  m_trigger_on_poly_entry = false;

  // Maintain a flag indicating whether the vehicle has entered
  // the polygon region. This value is only relevant if the 
  // trigger_on_poly_entry flag is set to be true.
  m_poly_entry_made = false;

  // Declare the amount of time required for the vehicle to be 
  // within the polygon region before the polygon containment 
  // condition is enforced. This value is only relevant if the 
  // trigger_on_poly_entry flag is set to be true.
  m_trigger_entry_time = 1.0;

  // Declare the amount of time required for the vehicle to be 
  // outside the polygon region before the polygon containment 
  // condition triggers a declaration of emergency. Setting this
  // to be non-zero may be useful if the position sensor (GPS) 
  // occasionally has a whacky single position reading outside
  // the polygon region.
  m_trigger_exit_time = 0.5;

  // 2011-03 SK: the buffer zone inside the vertex for which we
  //             bounce back
  m_bounce_buffer = 0;
  // 2011-04 SK: depth buffer
  m_depth_buffer = 0;
  // 2011-04 SK: no_zone factor to calculate no_zone from buffer (atm depth only)
  m_no_zone_factor = 0.2;

  // 2011-03 SK: I prefer to initialize these, avoid strange instantiations
  m_hint_edge_size = 2;
  m_hint_vertex_size = 2;

  // 2011-03 SK: IvP params as behaviour parameters
  // 2011-08 SK: based on feedback MOOS-DAWG, opened up to about 180deg,
  //             icw summitdelta not too big -> more angles ok
  m_peakwidthCourse = 60.0;
  m_basewidthCourse = 60.0;
  m_peakwidthDepth = 4.0;
  m_basewidthDepth = 4.0;
  m_summitdelta = 10.0;
  m_minutil = 0.0; // static
  m_maxutil = 100.0; // static

  // 2011-04 SK: initialize dynamic weighting variable
  m_pwt = 0.0;
  
  // 2011-04 SK: depth or perimeter emergency? initaliaze to false (all ok)
  m_depth_emergency = false;
  m_perimeter_emergency = false;

  // Declare the variables we will need from the info_buffer
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING, NAV_SPEED");
  if(m_couple)
    addInfoVars("NAV_DEPTH, NAV_ALTITUDE");

  // 2011-03 SK: debugging output?
  m_debug = false;
}

//-----------------------------------------------------------
// Procedure: setParam
//     Notes: We expect the "waypoint" entries will be of the form
//            "xposition,yposition,tag" where the tag is an optional
//            descriptor of the waypoint.
//            The "radius" parameter indicates what it means to have
//            arrived at the waypoint.

bool BHV_OpRegionBounce::setParam(string param, string val) 
{
  if(IvPBehavior::setParam(param, val))
    return(true);

  // Typical line: polygon  = 0,0:0,100:100,0:100,100
  if(param == "polygon") {
    XYPolygon new_poly = string2Poly(val);
    if(!new_poly.is_convex())  // Should be convex - false otherwise
      return(false);
    m_polygon = new_poly;
    m_polygons.push_back(new_poly);
    return(true);
  }
  else if(param == "max_depth") {
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
  else if(param == "max_time") {
    double dval = atof(val.c_str());
    if((dval < 0) || (!isNumber(val)))
      return(false);
    m_max_time = dval;
    return(true);
  }
  else if(param == "trigger_entry_time") {
    double dval = atof(val.c_str());
    if((dval < 0) || (!isNumber(val)))
      return(false);
    m_trigger_entry_time = dval;
    if(m_trigger_entry_time > 0)
      m_trigger_on_poly_entry = true;
    return(true);
  }
  else if(param == "trigger_exit_time") {
    double dval = atof(val.c_str());
    if((dval < 0) || (!isNumber(val)))
      return(false);
    m_trigger_exit_time = dval;
    return(true);
  }
  else if(param == "visual_hints")  {
    vector<string> svector = parseStringQ(val, ',');
    unsigned int i, vsize = svector.size();
    for(i=0; i<vsize; i++) 
      handleVisualHint(svector[i]);
    return(true);
  }
  // 2011-03 SK: size of the buffer inside the vertex for bouncing
  else if(param == "bounce_buffer") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_bounce_buffer = dval;
    return(true);
  }
  // 2011-04 SK: size of the buffer for depth bouncing
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
  else if(param == "peakwidth_course") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_peakwidthCourse = dval;
    return(true);
  }
  else if(param == "basewidth_course") {
    double dval = atof(val.c_str());
    if((dval > 0) && (isNumber(val)))
      m_basewidthCourse = dval;
    return(true);
  }
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

IvPFunction *BHV_OpRegionBounce::onRunState() 
{
  // SK: initialize IvP functions to avoid crashing the helm
  m_headingIPFcounter = 0;
  m_ipfHeading = 0;
  m_ipfDepth = 0;
  // SK: reset priority weight to parameter value in case it got changed.
  m_pwt = m_priority_wt;

  if(m_couple) {
    // SK: ** 2011-05-11 Coupler not working properly, putting depth in separate bhv **
    // SK: reset previous depths and weights
    unsigned int i;
    for(i=0; i < 2; i++) {
      m_safe_depth[i] = 0.0;
      m_safe_depth_weights[i] = 0.0;
    }
  }

  // reset to no emergencies.
  m_depth_emergency = false;
  m_perimeter_emergency = false;

  // Each of the below calls will check their critical conditions
  // and post an error message if a violation is detected. The call
  // to postEMessage() also sets state_ok = false;
  setTimeStamps();
  polygonVerify();
  postPolyStatus();

  if(m_couple) {
    depthVerify();
    altitudeVerify();
    // SK: ** 2011-05-11 Coupler not working properly, putting depth in separate bhv **
    // 2011-04 SK: if depth change necessary, create objective function(s)
    if(m_depth_emergency)
      createDepthBounce();
  }
  IvPFunction *ipf = 0; // SK: initialize to avoid crashing the helm
  if(m_couple) {
    // SK: ** 2011-05-11 Coupler not working properly, putting depth in separate bhv **
    if(m_depth_emergency && m_perimeter_emergency) {
      ipf = combineFunctions();
    }
    else if(m_depth_emergency && !m_perimeter_emergency) {
      ipf = m_ipfDepth;
      ipf->setPWT(m_depth_pwt);
    }
    else if(m_perimeter_emergency && !m_depth_emergency){
    ipf = m_ipfHeading;
    ipf->setPWT(m_pwt);
    }
  }
  else {
    if(m_perimeter_emergency && !m_depth_emergency){
      ipf = m_ipfHeading;
      ipf->setPWT(m_pwt);
    }
  }

  timeoutVerify();
  postViewablePolygon();

  if(ipf) {
    if(m_debug) {
      std::cout << std::endl << "** Created IPF **" << std::endl;
      ipf->getPDMap()->getDomain().print();
      ipf->getPDMap()->print();
      std::cout << "PWT : " << ipf->getPWT();
      std::cout << std::endl;
    }
    return(ipf);
  }
  else {
    if(m_debug)
      std::cout << "** No IPF Created **" << std::endl;
    return NULL;
  }
}

//-----------------------------------------------------------
// Procedure: polygonVerify()
//      Note: Verify that polygon boundary hasn't been violated.
//    Return: void - An error message is communicated by a call to 
//            postEMessage().

void BHV_OpRegionBounce::polygonVerify()
{
  if(m_polygons.size() == 0)
    return;

  bool ok1, ok2;
  double osX = getBufferDoubleVal("NAV_X", ok1);
  double osY = getBufferDoubleVal("NAV_Y", ok2);

  // Must get ownship position from InfoBuffer
  if(!ok1 || !ok2) {
    string msg = "No ownship info in info_buffer";
    postEMessage(msg);
    return;
  }

  bool contained = true;
  unsigned int i, vsize = m_polygons.size();
  for(i=0; i<vsize; i++) {
    if(!m_polygons[i].contains(osX, osY))
      contained = false;
  }

  // Determine the accumulated time within the polygon
  if(contained) {
    m_secs_out_poly = 0;
    if(m_previously_in_poly)
      m_secs_in_poly += m_delta_time;
    else
      m_secs_in_poly = 0;
    m_previously_in_poly = true;
  }
  // Determine the accumulated time outside the polygon
  if(!contained) {
    m_secs_in_poly = 0;
    if(!m_previously_in_poly)
      m_secs_out_poly += m_delta_time;
    else
      m_secs_out_poly = 0;
    m_previously_in_poly = false;
  }

  if(m_debug) {
    cout << "---------------------------" << endl;
    cout << "Previously In - " << m_previously_in_poly << endl;
    cout << "Contained -     " << contained << endl;
    cout << "Secs In  Poly - " << m_secs_in_poly << endl;
    cout << "Secs Out Poly - " << m_secs_out_poly << endl;
    cout << "Entry_Made:     " << m_poly_entry_made << endl;
  }

  // Case 1: Vehicle in polygon. Check to see if its been
  //         in the poly long enough to be considered an 
  //         official entry into the polygon.
  if(contained) {
    if(m_secs_in_poly >= m_trigger_entry_time) {
      m_poly_entry_made = true;

      // 2011-03 SK:
      // * calculate distance and angle vehicle to polygon
      // * based on geometry polygon, give a big preference for a
      //   heading orthogonal to polygon vertex
      // * base weight on distance to polygon and buffer zone param
      // all: only if still contained, keep BHV_ERROR for if not contained, 
      // hopefully this will never happen after setting up a buffer though.
      unsigned int i;
      unsigned int psize = m_polygons.size();
      if(m_debug)
        std::cout << "Amount of OpRegion polygons: " << psize << std::endl;
      for(i=0; i<psize; i++) {
        if(m_debug)
          std::cout << std::endl << "For polygon: " << i << std::endl;
        if(m_polygons[i].contains(osX, osY)) {
          // For the polygon that contains the vehicle 
          //  (assuming for now that there is only one)
          // Check if the AUV is close to a vertex
          double distToPoly = m_polygons[i].dist_to_poly(osX, osY);
          
          if(distToPoly < m_bounce_buffer) { // BOUNCE!
            m_perimeter_emergency = true;
            createCourseBounce(osX, osY, i);
          }
        }
      }
      // SK: now produce just one IPF function (and weight) for course
      if(m_perimeter_emergency)
        combineCourseIPFs();
    }
    return;
  }

  // Case 2: Vehicle not in polygon and no prior polygon
  //         entry is required for triggering emergency flag.
  //         Return based on accumulated time outside of poly.
  if(!contained && !m_trigger_on_poly_entry)
    if(m_secs_out_poly < m_trigger_exit_time)
      return;

  // Case 3: Vehicle not in polygon, poly entry is needed to
  //         trigger emergency, but no entry has been made yet.
  if(!contained && m_trigger_on_poly_entry && !m_poly_entry_made)
    return;

  // Case 4: Vehicle not in polygon, poly entry is needed to
  //         trigger emergency, and previous entry detected.
  //         Return based on accumulated time outside of poly.
  if(!contained && m_trigger_on_poly_entry && m_poly_entry_made)
    if(m_secs_out_poly < m_trigger_exit_time)
      return;

  // All verification cases failed. Post an error message and
  // return verification = false;
  string emsg = "BHV_OpRegionBounce Polygon containment failure: ";
  emsg += " x=" + doubleToString(osX);
  emsg += " y=" + doubleToString(osY);
  postEMessage(emsg);
}

//-----------------------------------------------------------
// Procedure: createCourseBounce
// added 2011-03 SK
//   Purpose: To construct the course objective function in case of
//            a bounce
//   Return:  void (saves to class var)

void BHV_OpRegionBounce::createCourseBounce(double osX, double osY, unsigned int polynr)
{
  XYSegList copiedSeglist = m_polygons[polynr].exportSegList(0,0);
  unsigned int vsize = copiedSeglist.size();

  // Check for each vertex whether the distance to it is smaller than the defined buffer
  unsigned int ix, ii;
  for(ix=0; ix<vsize; ix++) {
    // ii = next vertex
    ii = ix+1;
    if(ii == vsize)
      ii = 0;

    // Get indices for current and next vertex
    double x1, x2, y1, y2;
    x1 = copiedSeglist.get_vx(ix);
    x2 = copiedSeglist.get_vx(ii);
    y1 = copiedSeglist.get_vy(ix);
    y2 = copiedSeglist.get_vy(ii);
    // Now let's actually use these indices, as opposed to what's done in XYPolygon.cpp :P
    double idist = distPointToSeg(x1, y1, x2, y2, osX, osY);

    // if within buffer, create angle and weight for (later) zaic_peak construction
    if(idist < m_bounce_buffer) {
      // Line segment: x1, y1, x2, y2
      // This segment requires opposing forces. 

      // Calculate angle of line segment, from North
      m_course_angles[m_headingIPFcounter] = relAng(x1,y1,x2,y2);
      // Opposing force orthogonal to line segment
      m_course_angles[m_headingIPFcounter] += 90.0;
      m_course_angles[m_headingIPFcounter] = angle360( m_course_angles[m_headingIPFcounter] );

      // First part of equation makes weight be between 0 and 1.
      //   Because of above if statement(s), it is impossible that it
      //   would be smaller or bigger.
      //   Subtracting from 1 inverts it, far = ok, close = bad: linear function
      // Then we multiply by maxutil to convert it to the 0-100 interval (std maxutil).
      double no_zone = m_bounce_buffer*m_no_zone_factor;
      m_course_weights[m_headingIPFcounter] = (1-((idist-no_zone)/m_bounce_buffer))*m_maxutil;
      if(m_course_weights[m_headingIPFcounter] > m_maxutil)
        m_course_weights[m_headingIPFcounter] = m_maxutil;

      if(m_debug)
        std::cout << " Bounce! angle: " << m_course_angles[m_headingIPFcounter] << " @ " << m_course_weights[m_headingIPFcounter] << std::endl;

      m_headingIPFcounter++;
    }
  }//for(ix=0; ix<vsize; ix++)
}

//-----------------------------------------------------------
// Procedure: combineCourseIPFs()
// added 2011-04 SK
//      Note: If there are multiple polygons, we need to combine
//            the generated functions.
//    Return: void (saves to class var)

void BHV_OpRegionBounce::combineCourseIPFs() {
  unsigned int psize = m_polygons.size();

  if(m_debug) {
    std::cout << std::endl << "** BHV_OpRegionBounce::combineCourseIPFs" << std::endl;
    std::cout << " m_headingIPFcounter: " << m_headingIPFcounter << std::endl;
  }
  
  unsigned int ix;
  double maxWeight = 0.0;
  for(ix=0; ix<m_headingIPFcounter; ix++) {
    // check every created opposing force to determine max weight
    if(m_course_weights[ix] > maxWeight)
      maxWeight = m_course_weights[ix];
  }
  
  // Rescale all weights so that the max weight is 100 (maxutil), 
  //  then with the inverse factor scale the pwt. 
  // This will make it more clear what is going on in the pHelmIvP
  //  output, avoids the IvP plotter visualization problem, and seems
  //  to be the general approach to dynamic importance of bhv.
  // Nb. all weights should already be within 0-100 interval, no matter
  //     what polygon they belong to, therefore of equal importance so
  //     they can be scaled and combined here without problems
  //     (for example as encountered with different initial scaling)
  double factor = m_maxutil/maxWeight; 
  for(ix = 0; ix<m_headingIPFcounter; ix++) {
    m_course_weights[ix] *= factor;
  }
  // scale the pwt inverse to above scaling
  m_pwt = m_priority_wt/factor;

  // Create objective function that will include all opposing angles
  ZAIC_PEAK zaicCourse(m_domain, "course");
  bool prior = false;
  unsigned int index = 0;
  for(ix = 0; ix<m_headingIPFcounter; ix++) {
    // Params: summit, peakwidth, basewidth, summitdelta, minutil, maxutil, index=0
    // no need to worry about (summitdelta > maxutil) :: see doc TR-2009-037
    //
    // Add the opposing angle as the summit, use the weight as maxutil.
    if(m_course_weights[ix] > 0.0) {
      if(!prior) {
        zaicCourse.setParams(m_course_angles[ix], m_peakwidthCourse, m_basewidthCourse, m_summitdelta, m_minutil, m_course_weights[ix]);
        prior = true;
      }
      else {
        index = zaicCourse.addComponent();
        zaicCourse.setParams(m_course_angles[ix], m_peakwidthCourse, m_basewidthCourse, m_summitdelta, m_minutil, m_course_weights[ix], index);
      }
    }
  }
  zaicCourse.setValueWrap(true);
  zaicCourse.setSummitInsist(true);

  // debug output
  if(m_debug) {
    std::string zaic_warnings = zaicCourse.getWarnings();
    if(zaic_warnings != "") 
      postWMessage(zaic_warnings);

    for(ix = 0; ix<m_headingIPFcounter; ix++) {
      std::cout << "  angle[" << ix << "] " << m_course_angles[ix] << " @  " << m_course_weights[ix] << std::endl;
    }
    std::cout << "  m_pwt: " << m_pwt << std::endl;
  }

  // save function

  // extractIvPFunction(false): combine using sum of values <- wanted
  //                   (true) : take the max value of all
  m_ipfHeading = zaicCourse.extractIvPFunction(false);
  
  if(m_debug && m_ipfHeading)
    m_ipfHeading->getPDMap()->print();

  if(m_debug)
    std::cout << "** BHV_OpRegionBounce::combineCourseIPFs finished" << std::endl;  
}


//-----------------------------------------------------------
// Procedure: postPolyStatus
//   Purpose: To calculate information about impending OpRegionBounce
//            violations. It calculates and posts the following
//            variables:
//        (1) OPREG_TRAJECTORY_PERIM_ETA: (double) 
//            The time in seconds until the vehicle exits the 
//            polygon containment region if it stays on the 
//            current trajectory.
//        (2) OPREG_TRAJECTORY_PERIM_DIST: (double)
//            Distance in meters until the vehicle exits the 
//            polygon containment region if it stays on the 
//            current trajectory.
//        (3) OPREG_ABSOLUTE_PERIM_ETA: (double) 
//            Time in seconds until the vehicle would exit the
//            polygon if it were to take the shortest path at
//            top vehicle speed.
//        (4) OPREG_ABSOLUTE_PERIM_DIST: (double) 
//            Distance in meters between the vehicle and the 
//            polygon perimeter regardless of current heading
//        (5) OPREG_TIME_REMAINING: (double)
//            Time in seconds before a OpRegionBounce timeout would
//            occur. If max_time=0, then no such message posted.
//   
//    Return: void

void BHV_OpRegionBounce::postPolyStatus()
{
  bool ok1, ok2, ok3, ok4;
  double osX   = getBufferDoubleVal("NAV_X", ok1);
  double osY   = getBufferDoubleVal("NAV_Y", ok2);
  double osSPD = getBufferDoubleVal("NAV_SPEED", ok3);
  double osHDG = getBufferDoubleVal("NAV_HEADING", ok4);

  string msg;
  if(!ok1) 
    msg = "No ownship NAV_X (" + m_us_name + ") in info_buffer";
  if(!ok2) 
    msg = "No ownship NAV_Y (" + m_us_name + ") in info_buffer";
  if(!ok3) 
    msg = "No ownship NAV_SPEED (" + m_us_name + ") in info_buffer";
  if(!ok4) 
    msg = "No ownship NAV_HEADING (" + m_us_name + ") in info_buffer";

  // Must get ownship position from InfoBuffer
  if(!ok1 || !ok2 || !ok3 || !ok4) {
    postEMessage(msg);
    return;
  }

  // Must find the top vehicle speed in the behavior ivp domain
  int index = m_domain.getIndex("speed");
  if(index == -1) {
    string msg = "No Top-Speed info found in the decision domain";
    postEMessage(msg);
    return;
  }
  double osTopSpeed = m_domain.getVarHigh(index);
  
  // Calculate the time and the distance to the perimeter along the
  // current heading (CH).
  double trajectory_perim_dist = m_polygon.dist_to_poly(osX, osY, osHDG);
  double trajectory_perim_eta = 0;
  if(osSPD > 0)
    trajectory_perim_eta = trajectory_perim_dist / osSPD;
  
  // post the distance at integer precision unless close to zero
  if(trajectory_perim_dist <= 1)
    postMessage("OPREG_TRAJECTORY_PERIM_DIST", trajectory_perim_dist);
  else  
    postIntMessage("OPREG_TRAJECTORY_PERIM_DIST", trajectory_perim_dist);
  postIntMessage("OPREG_TRAJECTORY_PERIM_ETA",  trajectory_perim_eta);
  
  // Calculate the absolute (ABS) distance and ETA to the perimeter.
  double absolute_perim_dist = m_polygon.dist_to_poly(osX, osY);
  double absolute_perim_eta  = 0;
  if(osTopSpeed > 0)
    absolute_perim_eta  = absolute_perim_dist / osTopSpeed;
  
  // post the distance at integer precision unless close to zero
  if(absolute_perim_dist <= 1)
    postMessage("OPREG_ABSOLUTE_PERIM_DIST", absolute_perim_dist);
  else
    postIntMessage("OPREG_ABSOLUTE_PERIM_DIST", absolute_perim_dist);
  postIntMessage("OPREG_ABSOLUTE_PERIM_ETA",  absolute_perim_eta);
  
  if(m_max_time > 0) {
    double remaining_time = m_max_time - m_elapsed_time;
    if(remaining_time < 0)
      remaining_time = 0;
    postMessage("OPREG_TIME_REMAINING", remaining_time);
  }
}

//-----------------------------------------------------------
// Procedure: depthVerify()
//      Note: Verify that the depth limit hasn't been violated.
//    Return: void - An error message is communicated by a call to 
//            postEMessage().

void BHV_OpRegionBounce::depthVerify()
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

  if(m_couple) {
    // SK: ** 2011-05-11 Coupler not working properly, putting depth in separate bhv **
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
  }

  if(depth > m_max_depth) {
    string emsg = "OpRegionBounce Depth failure: max:";
    emsg += doubleToString(m_max_depth);
    emsg += " detected:" + doubleToString(depth);
    postEMessage(emsg);
  }
}

//-----------------------------------------------------------
// Procedure: altitudeVerify()
//      Note: Verify that the altitude limit hasn't been violated.
//    Return: void - An error message is communicated by a call to 
//            postEMessage().

void BHV_OpRegionBounce::altitudeVerify()
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

  if(m_couple) {
    // SK: ** 2011-05-11 Coupler not working properly, putting depth in separate bhv **
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
  }

  if(curr_altitude < m_min_altitude) {
    string emsg = "OpRegionBounce Altitude failure: Min-Altitude:";
    emsg += doubleToString(m_min_altitude);
    emsg += "  Detected Altitude: ";
    emsg += doubleToString(curr_altitude);
    postEMessage(emsg);
  }
}

//-----------------------------------------------------------
// Procedure: createDepthBounce()
// added 2011-04 SK
//   Purpose: To construct the depth objective function in case of
//            a bounce
//   Return:  void (saves to class var)
void BHV_OpRegionBounce::createDepthBounce()
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
  if(m_priority_wt > m_maxutil) { 
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

//-----------------------------------------------------------
// Procedure: combineFunctions()
// added 2011-04 SK
//      Note: Calculate parameters for, and combine course and 
//            depth IvP functions.
//    Return: resulting IvPFunction

IvPFunction* BHV_OpRegionBounce::combineFunctions() {
  // nb. this is only called if couple=true
  if(m_debug)
    std::cout << std::endl << "** BHV_OpRegionBounce::combineFunctions" << std::endl;

  double factor = 0.0;
  double rel_depth_pwt = 0.0;
  double rel_course_pwt = 0.0;
  double final_pwt = 0.0;

  IvPFunction* ipf;
  // Determine the highest factor, set to maxutil(100), scale everything
  // inversely, but set final_pwt to the original value, for scaling
  // the whole function back in shape.
  if(m_pwt > m_depth_pwt) {
    // scale by m_pwt, nb. m_maxutil = 100.0
    factor = m_maxutil/m_pwt;
    rel_course_pwt = m_maxutil;
    rel_depth_pwt = m_depth_pwt*factor;
    final_pwt = m_pwt;
  }
  else if(m_depth_pwt > m_pwt) {
    // scale by depth pwt
    factor = m_maxutil/m_depth_pwt;
    rel_course_pwt = m_pwt*factor;
    rel_depth_pwt = m_maxutil;
    final_pwt = m_depth_pwt;
  }
  else {
    // if the same 
    if(rel_course_pwt == rel_depth_pwt) {
      if(rel_course_pwt > m_maxutil) {
        // rescale both to maxutil
        rel_course_pwt = m_maxutil;
        rel_depth_pwt = m_maxutil;
        // use course pwt (should be same as depth pwt) for calc final pwt
        factor = m_maxutil/m_pwt;
        final_pwt = m_pwt*factor;
      }
      else {
        // use current values - both are equal, smaller than maxutil
        rel_course_pwt = m_pwt;
        rel_depth_pwt = m_depth_pwt;
      }
    }
    else {
      // cannot think of any more situation, but just in case, let's return that to the user
      std::cout << " Forgetting a situation here!!" << std::endl;
      std::cout << " Values m_pwt, m_depth_pwt: " << m_pwt << "  " << m_depth_pwt << std::endl;
    }
  }

  // Couple depth and course objective functions:
  OF_Coupler coupler; // initialize coupler in case it is needed
  // when passing functions to the coupler, we can do the relative scaling.
  //  IvPFunction *couple(IvPFunction* ipf_one, IvPFunction* ipf_two, double pwt_one, double pwt_two);
  ipf = coupler.couple(m_ipfHeading, m_ipfDepth, rel_course_pwt, rel_depth_pwt);
  if(m_debug) {
    std::cout << " relative course weight:: " << rel_course_pwt << std::endl;
    std::cout << " relative depth weight:: " << rel_depth_pwt << std::endl;
  }
  ipf->setPWT(final_pwt);

  if(m_debug)
    std::cout << "** BHV_OpRegionBounce::combineFunctions finished" << std::endl;

  // return the final function
  return ipf;
}


//-----------------------------------------------------------
// Procedure: timeoutVerify()
//      Note: Verify that the timeout limit hasn't been violated.
//    Return: void - An error message is communicated by a call to 
//            postEMessage().

void BHV_OpRegionBounce::timeoutVerify()
{
  // If no max_time specified, return with no error message posted.
  if(m_max_time <= 0)
    return;

  if(m_elapsed_time > m_max_time) {
    string emsg = "OpRegionBounce timeout failure: MaxTime:";
    emsg += doubleToString(m_max_time);
    emsg += "  Elapsed Time: ";
    emsg += doubleToString(m_elapsed_time);
    postEMessage(emsg);
  }
}


//-----------------------------------------------------------
// Procedure: setTimeStamps()

void BHV_OpRegionBounce::setTimeStamps()
{
  // Grab current time from Info Buffer
  m_current_time = getBufferCurrTime();
  
  //cout << "Current Time -    " << delta_time << endl;
  //cout << "Previous Time -    " << delta_time << endl;

  // Calculate the Delta time since this behavior was invoked.
  // The delta time is 0 on first invocation.
  if(m_first_time) {
    m_start_time = m_current_time;
    m_delta_time = 0;
    m_first_time = false;
  }
  else
    m_delta_time = m_current_time - m_previous_time;

  // No longer need to access previous time. Set it now for
  // access on the next invocation of this behavior.
  m_previous_time = m_current_time;

  m_elapsed_time = m_current_time - m_start_time;
}


//-----------------------------------------------------------
// Procedure: postViewablePolygon()
//      Note: Even if the polygon is posted on each iteration, the
//            helm will filter out unnecessary duplicate posts.

void BHV_OpRegionBounce::postViewablePolygon()
{
  if(m_polygon.size() == 0)
    return;
  XYPolygon poly_duplicate = m_polygon;
  if(m_hint_vertex_color != "")
    poly_duplicate.set_color("vertex",m_hint_vertex_color);
  if(m_hint_edge_color != "")
    poly_duplicate.set_color("edge",m_hint_edge_color);
  if(m_hint_edge_size >= 0)
    poly_duplicate.set_edge_size(m_hint_edge_size);
  if(m_hint_vertex_size >= 0)
    poly_duplicate.set_vertex_size(m_hint_vertex_size);

  string poly_spec = poly_duplicate.get_spec();
  postMessage("VIEW_POLYGON", poly_spec);
}

//-----------------------------------------------------------
// Procedure: postErasablePolygon()
//      Note: Even if the polygon is posted on each iteration, the
//            helm will filter out unnecessary duplicate posts.

void BHV_OpRegionBounce::postErasablePolygon()
{
  if(m_polygon.size() == 0)
    return;
  XYPolygon poly_duplicate = m_polygon;
  poly_duplicate.set_active(false);
  string poly_spec = poly_duplicate.get_spec();
  postMessage("VIEW_POLYGON", poly_spec);
}

//-----------------------------------------------------------
// Procedure: handleVisualHint()

void BHV_OpRegionBounce::handleVisualHint(string hint)
{
  string param = tolower(stripBlankEnds(biteString(hint, '=')));
  string value = stripBlankEnds(hint);
  double dval  = atof(value.c_str());

  if((param == "vertex_color") && isColor(value))
    m_hint_vertex_color = value;
  else if((param == "edge_color") && isColor(value))
    m_hint_edge_color = value;
  else if((param == "edge_size") && isNumber(value) && (dval >= 0))
    m_hint_edge_size = dval;
  else if((param == "vertex_size") && isNumber(value) && (dval >= 0))
    m_hint_vertex_size = dval;
}
