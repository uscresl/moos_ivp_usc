##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt
# DATE:        2010/09/07
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend source directory
##############################################################################

#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(./utils)

#============================================================================
# Add CMake modules that do not come with the system
#============================================================================
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake_modules/")

#============================================================================
# List the subdirectories to build...
#============================================================================
# utils dir
ADD_SUBDIRECTORY(utils)

# standard dirs
ADD_SUBDIRECTORY(lib_behaviors-test)
ADD_SUBDIRECTORY(pXRelayTest)

# template dir
ADD_SUBDIRECTORY(pTemplate)

# for all behaviors
ADD_SUBDIRECTORY(lib_behaviors)

# Robotics '13 class project Supreeth & Stephanie
ADD_SUBDIRECTORY(uFldHazardMgr)
ADD_SUBDIRECTORY(pHazardPath)

# CS599 Coord Mob Rob '14 class project Stephanie
find_package( PkgConfig )
pkg_check_modules( EIGEN3 eigen3 )
if ( EIGEN3_FOUND )
  message("EIGEN3_INCLUDE_DIRS: " ${EIGEN3_INCLUDE_DIRS})
  include_directories( ${EIGEN3_INCLUDE_DIRS} )
  ADD_SUBDIRECTORY(pSelectFormation)
  ADD_SUBDIRECTORY(pPositionInFormation)
endif()
ADD_SUBDIRECTORY(pFormationWptUpdater)
ADD_SUBDIRECTORY(pNumVehicles)

# find boost for multiple processes
SET(BOOST_MIN_VERSION "1.56.0")
find_package(Boost)

# get boost 1.55
#SET(BOOST_ROOT /usr/local/include/boost)
#SET(BOOST_LIBRARYDIR /usr/local/lib)
#SET(Boost_USE_STATIC_LIBS ON)
#SET(Boost_USE_STATIC_RUNTIME ON)
#SET(Boost_DEBUG ON)
#find_package(Boost 1.54)# REQUIRED)# COMPONENTS geometry)
#if ( Boost_FOUND )
#  if (Boost_VERSION 1.54)
#    ADD_SUBDIRECTORY(pMaxFormationWidth)
#  endif()
#endif()

# Nov'15: add simulation biology
if ( Boost_FOUND )
  ADD_SUBDIRECTORY(uSimBioSensor)
endif()

# Jan'16: add process to determine sample points for GP
# need MOOS Geodesy for conversion x/y/lon/lat
find_package(MOOSGeodesy)
if ( MOOSGeodesy_FOUND )
  include_directories(${MOOSGeodesy_INCLUDE_DIRS})
  link_directories(${MOOSGeodesy_LIBRARY_PATH})
  message("MOOSGeodesy_INCLUDE_DIRS:" ${MOOSGeodesy_INCLUDE_DIRS})
  message("MOOSGeodesy_LIBRARY_PATH:" ${MOOSGeodesy_LIBRARY_PATH})
  add_subdirectory(pSamplePoints)
  # Feb'16: add process to convert a lon/lat location to x/y wpt update
  add_subdirectory(pLonLatToWptUpdate)
endif()

# Jun'16: add process to republish data for easy import into acomms msg
add_subdirectory(pRePublishDataPtWithNav)

# Jan'16/Jun'16: add process for GP generation, Jun; now also dependent on geodesy
FIND_PACKAGE(LibGP)
if ( Boost_VERSION GREATER 105400 )
  message("   BOOST VERSION: " ${Boost_VERSION})
  set(BUILD_VORONOI)
  message("   Build VORONOI? YES")
else()
  message("   Build VORONOI? NO")
endif()

if ( LIBGP_FOUND AND MOOSGeodesy_FOUND AND Boost_FOUND )
#  if (Boost_VERSION 1.56)
#  message("     found LibGP")
#  message("     include dirs: " ${LIBGP_INCLUDE_DIRS})
#  message("     target link dirs: " ${LIBGP_LIBRARIES})
  include_directories(${LIBGP_INCLUDE_DIRS})
  include_directories(${MOOSGeodesy_INCLUDE_DIRS})
  link_directories(${MOOSGeodesy_LIBRARY_PATH})
  add_subdirectory(pGP)
  add_subdirectory(pGP_AUV)
#  endif()
else()
  message("LibGP not found. Did you set the GPLIB_LOCATION environment variable?")
endif()

# Sept'16: add process that can publish a certain var/val set,
#          X seconds after it received the publishment of another var/val set
add_subdirectory(pPublishAfterX)


##############################################################################
#                           END of CMakeLists.txt
##############################################################################
