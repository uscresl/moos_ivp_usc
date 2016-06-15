#--------------------------------------------------------
# The CMakeLists.txt for:         pRePublishDataPtWithNav
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  RePublishDataPtWithNav.cpp
  main.cpp
)

ADD_EXECUTABLE(pRePublishDataPtWithNav ${SRC})

TARGET_LINK_LIBRARIES(pRePublishDataPtWithNav
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

