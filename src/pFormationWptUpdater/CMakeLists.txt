#--------------------------------------------------------
# The CMakeLists.txt for:           pFormationWptUpdater
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  FormationWptUpdater.cpp
  main.cpp
)

ADD_EXECUTABLE(pFormationWptUpdater ${SRC})

TARGET_LINK_LIBRARIES(pFormationWptUpdater
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

