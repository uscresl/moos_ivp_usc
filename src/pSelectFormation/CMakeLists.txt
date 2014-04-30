#--------------------------------------------------------
# The CMakeLists.txt for:                pSelectFormation
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  SelectFormation.cpp
  main.cpp
)

ADD_EXECUTABLE(pSelectFormation ${SRC})

TARGET_LINK_LIBRARIES(pSelectFormation
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   uscutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)
