/*****************************************************************/
/*    NAME: Michael Benjamin and John Leonard                    */
/*    ORGN: NAVSEA Newport RI and MIT Cambridge MA               */
/*    FILE: BHV_OpRegionBounce.h                                 */
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

#ifndef BHV_OPREGIONBOUNCE_HEADER
#define BHV_OPREGIONBOUNCE_HEADER

#include "IvPBehavior.h"
#include "XYPolygon.h"

class BHV_OpRegionBounce : public IvPBehavior {
 public:
  BHV_OpRegionBounce(IvPDomain);
  ~BHV_OpRegionBounce() {};
  
  bool         setParam(std::string, std::string);
  IvPFunction* onRunState();
  void         onIdleState()     {postErasablePolygon();};
  void         onCompleteState() {postErasablePolygon();};

 protected:
  void      polygonVerify();
  void      postPolyStatus();
  void      depthVerify();
  void      altitudeVerify();
  void      timeoutVerify();
  void      setTimeStamps();
  void      handleVisualHint(std::string);
  void      postViewablePolygon();
  void      postErasablePolygon();
  // 2011-03/04 SK
  void      createCourseBounce(double, double, unsigned int);
  void      combineCourseIPFs();
  void      createDepthBounce();
  IvPFunction* combineFunctions();
  // end SK

 protected: // Parameters
  std::vector<XYPolygon> m_polygons;
  XYPolygon m_polygon;
  double    m_max_depth;
  double    m_min_altitude;
  double    m_max_time;
  double    m_trigger_entry_time;
  double    m_trigger_exit_time;
  bool      m_trigger_on_poly_entry;
  // 2011-03/04 SK
  double    m_bounce_buffer;
  double    m_peakwidthCourse;
  double    m_peakwidthDepth;
  double    m_basewidthCourse;
  double    m_basewidthDepth;
  double    m_summitdelta;
  double    m_minutil;
  double    m_maxutil;
  double    m_pwt;
  double    m_depth_pwt;
  double    m_depth_buffer;
  double    m_no_zone_factor;
  // end SK

  // Visual hints affecting properties of polygons/points
  std::string m_hint_vertex_color;
  std::string m_hint_edge_color;
  double      m_hint_vertex_size;
  double      m_hint_edge_size;

 protected: // State Variables
  bool      m_poly_entry_made;
  double    m_previous_time;  // Seconds
  double    m_current_time;
  double    m_delta_time;
  double    m_start_time;
  double    m_elapsed_time;

  double    m_secs_in_poly;
  double    m_secs_out_poly;

  bool      m_first_time;
  bool      m_previously_in_poly;
  
  // 2011-03/04/05 SK
  bool      m_couple;
  bool      m_debug;
  bool      m_depth_emergency;
  bool      m_perimeter_emergency;
  unsigned int m_headingIPFcounter;
  IvPFunction* m_ipfHeading;
  IvPFunction* m_ipfDepth;
  double    m_course_angles[1000];  // NB. static limit of 1000
                                    // should be enough ..
  double    m_course_weights[1000]; // NB. static limit of 1000
  double    m_safe_depth[2];
  double    m_safe_depth_weights[2];
  // end SK
};

extern "C" {
  IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_OpRegionBounce(domain);}
}

#endif
