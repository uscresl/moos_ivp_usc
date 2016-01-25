#--------------------------------------------------------
# The CMakeLists.txt for:                   pSamplePoints
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  SamplePoints.cpp
  main.cpp
)

ADD_EXECUTABLE(pSamplePoints ${SRC})

TARGET_LINK_LIBRARIES(pSamplePoints
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

