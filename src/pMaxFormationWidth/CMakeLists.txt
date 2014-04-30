#--------------------------------------------------------
# The CMakeLists.txt for:             pMaxFormationWidth
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  MaxFormationWidth.cpp
  main.cpp
)

ADD_EXECUTABLE(pMaxFormationWidth ${SRC})

include_directories(${Boost_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(pMaxFormationWidth
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   uscutil
   ${SYSTEM_LIBS}
   ${Boost_LIBRARIES}
   # whatever other libs you need
)

