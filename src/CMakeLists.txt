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
ADD_SUBDIRECTORY(pSelectFormation)
ADD_SUBDIRECTORY(pPositionInFormation)
ADD_SUBDIRECTORY(pFormationWptUpdater)

# get boost 1.55
SET(BOOST_ROOT /usr/local/include/boost)
SET(BOOST_LIBRARYDIR /usr/local/lib)
#SET(Boost_USE_STATIC_LIBS ON)
#SET(Boost_USE_STATIC_RUNTIME ON)
SET(Boost_DEBUG ON)
find_package(Boost 1.55 REQUIRED)# COMPONENTS geometry)
if (Boost_FOUND)
  ADD_SUBDIRECTORY(pMaxFormationWidth)
endif()


##############################################################################
#                           END of CMakeLists.txt
##############################################################################
