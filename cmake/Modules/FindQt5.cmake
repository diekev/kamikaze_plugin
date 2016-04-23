# - Find QT5 library
# Find the native QT5 includes and library
# This module defines
#  QT5_INCLUDE_DIRS, where to find qt5.h, Set when
#                    QT5 is found.
#  QT5_LIBRARIES, libraries to link against to use QT5.
#  QT5_ROOT_DIR, The base directory to search for QT5.
#                This can also be an environment variable.
#  QT5_FOUND, If false, do not try to use QT5.
#
# also defined, but not for general use are
#  QT5_LIBRARY, where to find the QT5 library.

# If QT5_ROOT_DIR was defined in the environment, use it.
IF(NOT QT5_ROOT_DIR AND NOT $ENV{QT5_ROOT_DIR} STREQUAL "")
	set(QT5_ROOT_DIR $ENV{QT5_ROOT_DIR})
endif()

set(_qt5_SEARCH_DIRS
	${QT5_ROOT_DIR}
)

find_path(QT5_INCLUDE_DIR
	NAMES
		include/QtCore/QString
	HINTS
		${_qt5_SEARCH_DIRS}
	PATH_SUFFIXES
		include
)

find_library(QT5_LIBRARY
	NAMES
		Qt5Core
	HINTS
		${_qt5_SEARCH_DIRS}
	PATH_SUFFIXES
		lib64 lib
)

# handle the QUIETLY and REQUIRED arguments and set QT5_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(QT5 DEFAULT_MSG
		QT5_LIBRARY QT5_INCLUDE_DIR)

if(QT5_FOUND)
	set(QT5_LIBRARIES ${QT5_LIBRARY})
	set(QT5_INCLUDE_DIRS ${QT5_INCLUDE_DIR} ${QT5_INCLUDE_DIR}/include ${QT5_INCLUDE_DIR}/include/QtCore)
else()
	set(QT5_QT5_FOUND FALSE)
endif()

mark_as_advanced(
	QT5_INCLUDE_DIR
	QT5_LIBRARY
)

unset(_qt5_SEARCH_DIRS)
