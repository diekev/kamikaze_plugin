# - Find OpenEXR library
# Find the native OpenEXR includes and library
# This module defines
#  OPENEXR_INCLUDE_DIRS, where to find ImfXdr.h, etc. Set when
#                      OPENEXR_INCLUDE_DIR is found.
#  OPENEXR_LIBRARIES, libraries to link against to use OpenEXR.
#  OPENEXR_ROOT_DIR, The base directory to search for OpenEXR.
#                      This can also be an environment variable.
#  OPENEXR_FOUND, If false, do not try to use OpenEXR.
#
# For individual library access these advanced settings are available
#  OPENEXR_HALF_LIBRARY, Path to Half library
#  OPENEXR_IEX_LIBRARY, Path to Half library
#  OPENEXR_ILMIMF_LIBRARY, Path to Ilmimf library
#  OPENEXR_ILMTHREAD_LIBRARY, Path to IlmThread library
#  OPENEXR_IMATH_LIBRARY, Path to Imath library
#
# also defined, but not for general use are
#  OPENEXR_LIBRARY, where to find the OpenEXR library.

#=============================================================================
# Copyright 2011 Blender Foundation.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

# If OPENEXR_ROOT_DIR was defined in the environment, use it.
IF(NOT OPENEXR_ROOT_DIR AND NOT $ENV{OPENEXR_ROOT_DIR} STREQUAL "")
	set(OPENEXR_ROOT_DIR $ENV{OPENEXR_ROOT_DIR})
endif()

# Old versions (before 2.0?) do not have any version string, just assuming this should be fine though.
set(_openexr_libs_ver_init "2.0")

set(_openexr_FIND_COMPONENTS
	Half
	Iex
	IlmImf
	IlmThread
	Imath
)

set(_openexr_SEARCH_DIRS
	${OPENEXR_ROOT_DIR}
	/opt/lib/openexr
)

find_path(OPENEXR_INCLUDE_DIR
	NAMES
		OpenEXR/ImfXdr.h
	HINTS
		${_openexr_SEARCH_DIRS}
	PATH_SUFFIXES
		include
)

# If the headers were found, get the version from config file, if not already set.
if(OPENEXR_INCLUDE_DIR)
	if(NOT OPENEXR_VERSION)

		FIND_FILE(_openexr_CONFIG
			NAMES
				OpenEXRConfig.h
			PATHS
				"${OPENEXR_INCLUDE_DIR}"
				"${OPENEXR_INCLUDE_DIR}/OpenEXR"
			NO_DEFAULT_PATH
		)

		if(_openexr_CONFIG)
			file(STRINGS "${_openexr_CONFIG}" OPENEXR_BUILD_SPECIFICATION
					 REGEX "^[ \t]*#define[ \t]+OPENEXR_VERSION_STRING[ \t]+\"[.0-9]+\".*$")
		else()
			message(WARNING "Could not find \"OpenEXRConfig.h\" in \"${OPENEXR_INCLUDE_DIR}\"")
		endif()

		if(OPENEXR_BUILD_SPECIFICATION)
			message(STATUS "${OPENEXR_BUILD_SPECIFICATION}")
			string(REGEX REPLACE ".*#define[ \t]+OPENEXR_VERSION_STRING[ \t]+\"([.0-9]+)\".*"
						 "\\1" _openexr_libs_ver_init ${OPENEXR_BUILD_SPECIFICATION})
		else()
			message(WARNING "Could not determine ILMBase library version, assuming ${_openexr_libs_ver_init}.")
		endif()

		unset(_openexr_CONFIG CACHE)
	endif()
endif()

set("OPENEXR_VERSION" ${_openexr_libs_ver_init} CACHE STRING "Version of OpenEXR lib")
unset(_openexr_libs_ver_init)

string(REGEX REPLACE "([0-9]+)[.]([0-9]+).*" "\\1_\\2" _openexr_libs_ver ${OPENEXR_VERSION})

set(_openexr_LIBRARIES)
foreach(COMPONENT ${_openexr_FIND_COMPONENTS})
	string(TOUPPER ${COMPONENT} UPPERCOMPONENT)

	find_library(OPENEXR_${UPPERCOMPONENT}_LIBRARY
		NAMES
			${COMPONENT}-${_openexr_libs_ver} ${COMPONENT}
		HINTS
			${_openexr_SEARCH_DIRS}
		PATH_SUFFIXES
			lib64 lib
		)
	list(APPEND _openexr_LIBRARIES "${OPENEXR_${UPPERCOMPONENT}_LIBRARY}")
endforeach()

unset(_openexr_libs_ver)

# handle the QUIETLY and REQUIRED arguments and set OPENEXR_FOUND to TRUE if 
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenEXR  DEFAULT_MSG
		_openexr_LIBRARIES OPENEXR_INCLUDE_DIR)

if(OPENEXR_FOUND)
	set(OPENEXR_LIBRARIES ${_openexr_LIBRARIES})
	set(OPENEXR_INCLUDE_DIRS ${OPENEXR_INCLUDE_DIR})
endif()

mark_as_advanced(
	OPENEXR_INCLUDE_DIR
	OPENEXR_VERSION
)

foreach(COMPONENT ${_openexr_FIND_COMPONENTS})
	string(TOUPPER ${COMPONENT} UPPERCOMPONENT)
	mark_as_advanced(OPENEXR_${UPPERCOMPONENT}_LIBRARY)
endforeach()

unset(COMPONENT)
unset(UPPERCOMPONENT)
unset(_openexr_FIND_COMPONENTS)
unset(_openexr_LIBRARIES)
unset(_openexr_SEARCH_DIRS)
