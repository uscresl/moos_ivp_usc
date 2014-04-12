#--------------------------------------------------------
# The CMakeLists.txt for:                       pTemplate
# Author(s): Stephanie Kemna, USC
# Note: this is a template dir, so you can copy to start
#       making your MOOSApp
#--------------------------------------------------------

SET(SRC
  Template.cpp
  main.cpp
)

ADD_EXECUTABLE(pTemplate ${SRC})

TARGET_LINK_LIBRARIES(pTemplate
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)

