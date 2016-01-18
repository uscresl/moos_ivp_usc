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
  include_directories( ${EIGEN3_INCLUDE_DIRS} )
  ADD_SUBDIRECTORY(pSelectFormation)
  ADD_SUBDIRECTORY(pPositionInFormation)
endif()
ADD_SUBDIRECTORY(pFormationWptUpdater)
ADD_SUBDIRECTORY(pNumVehicles)

# get boost 1.55
SET(BOOST_ROOT /usr/local/include/boost)
SET(BOOST_LIBRARYDIR /usr/local/lib)
#SET(Boost_USE_STATIC_LIBS ON)
#SET(Boost_USE_STATIC_RUNTIME ON)
SET(Boost_DEBUG ON)
#find_package(Boost 1.54)# REQUIRED)# COMPONENTS geometry)
if (Boost_FOUND)
  if (Boost_VERSION 1.54)
    ADD_SUBDIRECTORY(pMaxFormationWidth)
  endif()
endif()

# Nov'15: add simulation biology
#FIND_PACKAGE(PythonLibs 2.7 REQUIRED) #3.2 REQUIRED)
FIND_PACKAGE(Flann)
FIND_PACKAGE(HDF5)
#if ( PYTHONLIBS_FOUND )
##  message("     found Python libs, version " ${PYTHONLIBS_VERSION_STRING})
##  message("     include dirs: " ${PYTHON_INCLUDE_DIR})
##  message("     target link libs: " ${PYTHON_LIBRARIES})
#  include_directories( ${PYTHON_INCLUDE_DIRS} )
#endif()
if ( FLANN_FOUND )
#  message("     found Flann libs")
#  message("     include dirs: " ${FLANN_INCLUDE_DIRS})
#  message("     target link libs: " ${FLANN_LIBRARIES})
  include_directories(${FLANN_INCLUDE_DIRS})
endif()
#if ( HDF5_FOUND )
#  message("     found HDF5")
#  message("     include dirs: " ${HDF5_INCLUDE_DIRS})
#  message("     target link dirs: " ${HDF5_LIBRARIES})
#endif()
if ( FLANN_FOUND AND HDF5_FOUND ) #PYTHONLIBS_FOUND AND
  ADD_SUBDIRECTORY(uSimBioSensor)
endif()

# Jan'16: add process for GP generation
FIND_PACKAGE(LibGP)
if ( LIBGP_FOUND )
#  message("     found LibGP")
#  message("     include dirs: " ${LIBGP_INCLUDE_DIRS})
#  message("     target link dirs: " ${LIBGP_LIBRARIES})
  include_directories(${LIBGP_INCLUDE_DIRS})
  add_subdirectory(pGP)
else()
  message("LibGP not found. Did you set the GPLIB_LOCATION environment variable?")
endif()


##############################################################################
#                           END of CMakeLists.txt
##############################################################################
