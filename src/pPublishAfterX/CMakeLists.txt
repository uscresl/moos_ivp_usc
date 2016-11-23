#--------------------------------------------------------
# The CMakeLists.txt for:                  pPublishAfterX
# Author(s): Stephanie Kemna, USC
#--------------------------------------------------------

SET(SRC
  PublishAfterX.cpp
  main.cpp
)

ADD_EXECUTABLE(pPublishAfterX ${SRC})

TARGET_LINK_LIBRARIES(pPublishAfterX
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   ${SYSTEM_LIBS}
   # whatever other libs you need
)
