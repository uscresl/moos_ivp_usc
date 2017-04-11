#--------------------------------------------------------
# The CMakeLists.txt for:                       pGP_AUV
# Author(s): Stephanie Kemna, USC
# Note: this is a template dir, so you can copy to start
#       making your MOOSApp
#--------------------------------------------------------

SET(SRC
  GP_AUV.cpp
  main.cpp
  GraphNode.cpp
)

ADD_EXECUTABLE(pGP_AUV ${SRC})

TARGET_LINK_LIBRARIES(pGP_AUV
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

