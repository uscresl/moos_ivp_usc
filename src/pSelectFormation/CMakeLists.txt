#--------------------------------------------------------
# The CMakeLists.txt for:                pSelectFormation
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  SelectFormation.cpp
  SelectFormation_utils.cpp
  main.cpp
)

ADD_EXECUTABLE(pSelectFormation ${SRC})

TARGET_LINK_LIBRARIES(pSelectFormation
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)
