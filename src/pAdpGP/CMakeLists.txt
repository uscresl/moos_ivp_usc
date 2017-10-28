#--------------------------------------------------------
# The CMakeLists.txt for:                          pAdpGP
# Author(s): Stephanie Kemna, USC
# Note: this is a template dir, so you can copy to start
#       making your MOOSApp
#--------------------------------------------------------

SET(SRC
  AdpGP.cpp
  main.cpp
)

ADD_EXECUTABLE(pAdpGP ${SRC})

target_compile_definitions(pAdpGP PRIVATE BUILD_VORONOI=${BUILD_VORONOI})

TARGET_LINK_LIBRARIES(pAdpGP
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

