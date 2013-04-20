#--------------------------------------------------------
# The CMakeLists.txt for:                       pHazardPath
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  HazardPath.cpp
  main.cpp
)

ADD_EXECUTABLE(pHazardPath ${SRC})

TARGET_LINK_LIBRARIES(pHazardPath
   ${MOOS_LIBRARIES}
   ufld_hazards
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
)

