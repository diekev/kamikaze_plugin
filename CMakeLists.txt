# ***** BEGIN GPL LICENSE BLOCK *****
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# The Original Code is Copyright (C) 2016 Kévin Dietrich.
# All rights reserved.
#
# ***** END GPL LICENSE BLOCK *****

# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8)

# ------------------------------------------------------------------------------

project(kamikaze_plugins)

# ------------------------------------------------------------------------------

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

# ------------------------------------------------------------------------------

set(CMAKE_BUILD_TYPE_INIT "Release")

# ------------------------------------------------------------------------------

add_compile_options(-std=c++14)

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-DGLM_FORCE_RADIANS)

# ------------------------------------------------------------------------------

find_package(Ego REQUIRED)
find_package(Kamikaze REQUIRED)
find_package(OpenEXR REQUIRED)
find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)
find_package(Qt5 REQUIRED)

include_directories(
	.
	${CMAKE_CURRENT_SOURCE_DIR}/../
	${EGO_INCLUDE_DIRS}
	${KAMIKAZE_INCLUDE_DIRS}
	${OPENEXR_INCLUDE_DIRS}
	${OPENVDB_INCLUDE_DIRS}
	${QT5_INCLUDE_DIRS}
)

set(OPENVDB_LIBRARIES
	${OPENEXR_LIBRARIES}
	${OPENVDB_LIBRARIES}
	${TBB_LIBRARIES}
)

set(VDB_PRIM_SRC
	levelset.cc
	main.cc
	util_openvdb.cc
	volume.cc
	volumebase.cc

	levelset.h
	util_openvdb.h
	util_openvdb_process.h
	volume.h
	volumebase.h
)

macro(define_plugin plugin_name source_files libraries)
	add_library(${plugin_name} SHARED ${source_files})
	install(TARGETS ${plugin_name} LIBRARY DESTINATION plugins)
	target_link_libraries(${plugin_name} ${libraries})
endmacro()

define_plugin(vdb_prim "${VDB_PRIM_SRC}" "${OPENVDB_LIBRARIES}")

set(KAMIKAZE_OPENVDB_LIBS
	${OPENVDB_LIBRARIES}
	vdb_prim
)

set(NODE_SRC
	node_openvdb_filter_level_set.cc
	node_openvdb_from_polygons.cc
	node_openvdb_noise.cc
	node_openvdb_platonic.cc
	node_openvdb_write.cc
)

foreach(file_name ${NODE_SRC})
	get_filename_component(file ${file_name} NAME_WE)
	define_plugin(${file} ${file_name} "${KAMIKAZE_OPENVDB_LIBS}")
endforeach()
