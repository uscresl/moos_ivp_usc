#--------------------------------------------------------
# The CMakeLists.txt for:             pLonLatToWptUpdate
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  LonLatToWptUpdate.cpp
  main.cpp
)

ADD_EXECUTABLE(pLonLatToWptUpdate ${SRC})

TARGET_LINK_LIBRARIES(pLonLatToWptUpdate
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

