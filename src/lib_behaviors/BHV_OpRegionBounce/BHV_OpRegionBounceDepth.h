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
/*                                                               */
/*    Name: Stephanie Kemna (SK) <kemna@usc.edu>                 */
/*    Organization: University of Southern California, LA, CA, US*/
/*    Date: May 24th, 2017                                       */
/*    Edits: added an averaging filter over last X measurements  */
/*           to avoid instant action on erroneous measurements   */
/*                                                               */
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

#ifndef BHV_OPREGIONBOUNCEDEPTH_HEADER
#define BHV_OPREGIONBOUNCEDEPTH_HEADER

#include "IvPBehavior.h"

class BHV_OpRegionBounceDepth : public IvPBehavior {
 public:
  BHV_OpRegionBounceDepth(IvPDomain);
  ~BHV_OpRegionBounceDepth() { delete[] m_last_altitudes; delete[] m_last_depths; };

  bool         setParam(std::string, std::string);
  IvPFunction* onRunState();
  void         onIdleState()     {};
  void         onCompleteState() {};

 protected:
  void      depthVerify();
  void      altitudeVerify();
  void      createDepthBounce();

 protected: // Parameters
  double    m_max_depth;
  double    m_min_altitude;

  double    m_peakwidthDepth;
  double    m_basewidthDepth;
  double    m_summitdelta;
  double    m_minutil;
  double    m_maxutil;
  double    m_depth_pwt;
  double    m_depth_buffer;
  double    m_no_zone_factor;
  size_t    m_avg_filter_size;

 protected: // State Variables
  bool      m_debug;
  bool      m_depth_emergency;
  IvPFunction* m_ipfDepth;
  double    m_safe_depth[2];
  double    m_safe_depth_weights[2];
  double *  m_last_altitudes;
  size_t    m_alt_count;
  double *  m_last_depths;
  size_t    m_depth_count;
};

extern "C" {
  IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_OpRegionBounceDepth(domain);}
}

#endif
