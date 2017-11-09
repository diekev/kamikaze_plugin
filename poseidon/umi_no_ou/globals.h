/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2015 Kévin Dietrich.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#pragma once

#include <openvdb/tools/PointIndexGrid.h>

enum {
	EMPTY = 0,
	FLUID = 1,
	SOLID = 2
};

struct SimulationGlobals {
	float dt;  /* time step */
	float dh;  /* voxel size, sometimes refered to as dx or dTau */
	int advection_scheme;
	float max_vel;
	openvdb::math::Transform xform;
};

struct SimulationFields {
	openvdb::VectorGrid::Ptr velocity;
	openvdb::ScalarGrid::Ptr density;
	openvdb::ScalarGrid::Ptr temperature;
	openvdb::ScalarGrid::Ptr pressure;
	openvdb::BoolGrid::Ptr obstacle;
	openvdb::Int32Grid::Ptr flags;
	openvdb::tools::PointIndexGrid::Ptr pindex;
};
