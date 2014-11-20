#--------------------------------------------------------
# The CMakeLists.txt for:                  lib_usc-utils 
# Author(s):                             Stephanie Kemna 
#--------------------------------------------------------

SET(SRC 
  calcDxDyOperators2h.cpp
  calcDxDyOperatorsStd.cpp
  deg2rad.cpp
  euclidDistance.cpp
  getDoubleFromCommaSeparatedString.cpp
  getDoubleFromNodeReport.cpp
  getStringFromNodeReport.cpp
  quadrant.cpp
  rad2deg.cpp
  trig_dx.cpp
  trig_dy.cpp
)

SET(HEADERS USCutils.h)

# Build Library
ADD_LIBRARY(uscutil ${SRC})
