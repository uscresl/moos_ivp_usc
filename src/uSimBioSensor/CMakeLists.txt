#--------------------------------------------------------
# The CMakeLists.txt for:                   uSimBioSensor
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  SimBioSensor.cpp
  main.cpp
)

ADD_EXECUTABLE(uSimBioSensor ${SRC})

TARGET_LINK_LIBRARIES(uSimBioSensor
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

