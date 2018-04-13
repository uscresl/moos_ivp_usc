#--------------------------------------------------------
# The CMakeLists.txt for:                       pGP_AUV
# Author(s): Stephanie Kemna, USC
# Note: this is a template dir, so you can copy to start
#       making your MOOSApp
#--------------------------------------------------------

SET(SRC
  gen_pp_test.cpp
        gen_pp.cpp
  GraphNode.cpp
)

ADD_EXECUTABLE(gen_pp_test ${SRC})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I /usr/local/include")

# add C++11 to compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package( PkgConfig )
pkg_check_modules( EIGEN3 eigen3 )
if ( EIGEN3_FOUND )
    message("   Eigen include dirs: " ${EIGEN3_INCLUDE_DIRS} )
    include_directories( ${EIGEN3_INCLUDE_DIRS} )
endif()

TARGET_LINK_LIBRARIES(gen_pp_test
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
   ${LIBGP_LIBRARIES}
   ${Boost_LIBRARIES}
)