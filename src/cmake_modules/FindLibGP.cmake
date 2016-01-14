###############################################################################
# Find libGP
#
# This sets the following variables:
# LIBGP_FOUND - True if LIBGP was found.
# LIBGP_INCLUDE_DIRS - Directories containing the LIBGP include files.
# LIBGP_LIBRARIES - Libraries needed to use LIBGP.
# LIBGP_DEFINITIONS - Compiler flags for LIBGP.
#
# This module depends on the environment variable "LIBGP_LOCATION".
# Make sure to export this (locally or via bashrc), before building
#

find_package(PkgConfig)
pkg_check_modules(PC_LIBGP QUIET libgp)
set(LIBGP_DEFINITIONS ${PC_LIBGP_CFLAGS_OTHER})

set(GP_LOCATION $ENV{GPLIB_LOCATION} )

find_path(LIBGP_INCLUDE_DIR gp.h
          HINTS ${PC_LIBGP_INCLUDEDIR} ${PC_LIBGP_INCLUDE_DIRS} ${GP_LOCATION}/include/ )

find_library(LIBGP_LIBRARY NAMES gp libgp
             HINTS ${PC_LIBGP_LIBDIR} ${PC_LIBGP_LIBRARY_DIRS} ${GP_LOCATION}/build/ )

set(LIBGP_LIBRARIES ${LIBGP_LIBRARY} )
set(LIBGP_INCLUDE_DIRS ${LIBGP_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibGP DEFAULT_MSG
                                  LIBGP_LIBRARY LIBGP_INCLUDE_DIR)

mark_as_advanced(LIBGP_INCLUDE_DIR LIBGP_LIBRARY )
