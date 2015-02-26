#--------------------------------------------------------
# The CMakeLists.txt for:                    pNumVehicles
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  NumVehicles.cpp
  main.cpp
)

ADD_EXECUTABLE(pNumVehicles ${SRC})

TARGET_LINK_LIBRARIES(pNumVehicles
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   uscutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

