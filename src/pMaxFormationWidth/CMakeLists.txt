#--------------------------------------------------------
# The CMakeLists.txt for:                       pMaxFormationWidth
# Author(s): Stephanie Kemna, USC
# Note: this is a template dir, so you can copy to start
#       making your MOOSApp
#--------------------------------------------------------

SET(SRC
  MaxFormationWidth.cpp
  MaxFormationWidth_utils.cpp
  main.cpp
)

ADD_EXECUTABLE(pMaxFormationWidth ${SRC})

include_directories(${Boost_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(pMaxFormationWidth
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   ${Boost_LIBRARIES}
   # whatever other libs you need
)

