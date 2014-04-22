#--------------------------------------------------------
# The CMakeLists.txt for:           pPositionInFormation
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  PositionInFormation.cpp
  main.cpp
)

ADD_EXECUTABLE(pPositionInFormation ${SRC})

TARGET_LINK_LIBRARIES(pPositionInFormation
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

