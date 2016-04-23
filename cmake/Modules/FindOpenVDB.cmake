# - Find OpenVDB library
# Find the native OPENVDB includes and library
# This module defines
#  OPENVDB_INCLUDE_DIRS, where to find openvdb.h, Set when
#                    OPENVDB is found.
#  OPENVDB_LIBRARIES, libraries to link against to use OPENVDB.
#  OPENVDB_ROOT_DIR, The base directory to search for OPENVDB.
#                This can also be an environment variable.
#  OPENVDB_FOUND, If false, do not try to use OPENVDB.
#
# also defined, but not for general use are
#  OPENVDB_LIBRARY, where to find the OPENVDB library.

# If OPENVDB_ROOT_DIR was defined in the environment, use it.
IF(NOT OPENVDB_ROOT_DIR AND NOT $ENV{OPENVDB_ROOT_DIR} STREQUAL "")
	set(OPENVDB_ROOT_DIR $ENV{OPENVDB_ROOT_DIR})
endif()

set(_openvdb_SEARCH_DIRS
	${OPENVDB_ROOT_DIR}
	/opt/lib/openvdb
)

find_path(OPENVDB_INCLUDE_DIR
	NAMES
		openvdb/openvdb.h
	HINTS
		${_openvdb_SEARCH_DIRS}
	PATH_SUFFIXES
		include
)

find_library(OPENVDB_LIBRARY
	NAMES
		openvdb
	HINTS
		${_openvdb_SEARCH_DIRS}
	PATH_SUFFIXES
		lib64 lib
)

# handle the QUIETLY and REQUIRED arguments and set OPENVDB_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OPENVDB DEFAULT_MSG
		OPENVDB_LIBRARY OPENVDB_INCLUDE_DIR)

if(OPENVDB_FOUND)
	set(OPENVDB_LIBRARIES ${OPENVDB_LIBRARY})
	set(OPENVDB_INCLUDE_DIRS ${OPENVDB_INCLUDE_DIR})
else()
	set(OPENVDB_FOUND FALSE)
endif()

mark_as_advanced(
	OPENVDB_INCLUDE_DIR
	OPENVDB_LIBRARY
)

unset(_openvdb_SEARCH_DIRS)
