#***************************************************************#
#    Name: Stephanie Kemna (SK) <kemna@nurc.nato.int>           #
#    Organization: NATO Undersea Research Centre, La Spezia, IT #
#    File: CMakeLists.txt                                       #
#    Date: May, 2011                                            #
#                                                               #
#    Copyright (C) 2011  NURC                                   #
#                                                               #
# This program is free software; you can redistribute it and/or #
# modify it under the terms of the GNU General Public License   #
# as published by the Free Software Foundation; either version  #
# 3 of the License, or (at your option) any later version.      #
#                                                               #
# This program is distributed in the hope that it will be       #
# useful, but WITHOUT ANY WARRANTY; without even the implied    #
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       #
# PURPOSE. See the GNU General Public License for more details. #
#                                                               #
# You should have received a copy of the GNU General Public     #
# License along with this program.                              #
# If not, see <http://www.gnu.org/licenses/                     #
#                                                               #
#***************************************************************#
#-------------------------------------------------------------
# The CMakeLists.txt for:                   BHV_ORB/BHV_ORBD
#-------------------------------------------------------------

SET(CMAKE_VERBOSE_MAKEFILE ON)

GET_FILENAME_COMPONENT(KEMNA_SRC_DIR_ABS .. ABSOLUTE)

FILE(GLOB IVP_INCLUDE_DIRS 
   ${IVP_SRC_DIR_ABSOLUTE}/lib_behaviors
   ${IVP_SRC_DIR_ABSOLUTE}/lib_ivpbuild
   ${IVP_SRC_DIR_ABSOLUTE}/lib_logic
   ${IVP_SRC_DIR_ABSOLUTE}/lib_ivpcore 
   ${IVP_SRC_DIR_ABSOLUTE}/lib_bhvutil
   ${IVP_SRC_DIR_ABSOLUTE}/lib_mbutil
   ${IVP_SRC_DIR_ABSOLUTE}/lib_geometry
   )

INCLUDE_DIRECTORIES(${IVP_INCLUDE_DIRS})

#-------------------------------------------------------------
#                                         BHV_OpRegionBounce
#-------------------------------------------------------------
#ADD_BHV(BHV_OpRegionBounce)
ADD_LIBRARY(BHV_OpRegionBounce SHARED BHV_OpRegionBounce.cpp)
TARGET_LINK_LIBRARIES(BHV_OpRegionBounce
  behaviors ivpbuild logic ivpcore bhvutil mbutil geometry)

#-------------------------------------------------------------

#-------------------------------------------------------------
#                                    BHV_OpRegionBounceDepth
#-------------------------------------------------------------
#ADD_BHV(BHV_OpRegionBounceDepth)
ADD_LIBRARY(BHV_OpRegionBounceDepth SHARED BHV_OpRegionBounceDepth.cpp)
TARGET_LINK_LIBRARIES(BHV_OpRegionBounceDepth
  behaviors ivpbuild logic ivpcore bhvutil mbutil geometry)
#-------------------------------------------------------------
