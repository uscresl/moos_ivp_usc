#*****************************************************************#
#*    Name: Stephanie Kemna,                                     *#
#*    Orgn: University of Southern California, LA, CA            *#
#*    File: CMakeLists.txt                                       *#
#*    Date: Nov 25, 2015                                         *#
#*****************************************************************#

#-------------------------------------------------------------
# The CMakeLists.txt for:                     BHV_YoYo
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
#                                             BHV_YoYo
#-------------------------------------------------------------
ADD_LIBRARY(BHV_YoYo SHARED BHV_YoYo.cpp)
TARGET_LINK_LIBRARIES(BHV_YoYo
  behaviors ivpbuild logic ivpcore bhvutil mbutil geometry)

#-------------------------------------------------------------
