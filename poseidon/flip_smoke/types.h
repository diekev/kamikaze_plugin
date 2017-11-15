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

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/PointIndexGrid.h>

/* ******************************** Accessors ******************************** */

using ScalarAccessor = openvdb::ScalarGrid::Accessor;
using VectorAccessor = openvdb::VectorGrid::Accessor;

/* ********************************* Samplers ******************************** */

using PointScalarSampler  = openvdb::tools::GridSampler<ScalarAccessor, openvdb::tools::PointSampler>;
using PointVectorSampler  = openvdb::tools::GridSampler<VectorAccessor, openvdb::tools::StaggeredPointSampler>;
using LinearScalarSampler = openvdb::tools::GridSampler<ScalarAccessor, openvdb::tools::BoxSampler>;
using LinearVectorSampler = openvdb::tools::GridSampler<VectorAccessor, openvdb::tools::StaggeredBoxSampler>;

/* **************************** Special Grid Types *************************** */

using ParticleGrid = openvdb::tools::PointIndexGrid;
using FlagGrid = openvdb::Int32Grid;

enum CellType {
	TypeNone     = -1,
	TypeFluid    = 0,
	TypeObstacle = 1,
	TypeEmpty    = 2,
	TypeInflow   = 3,
	TypeOutflow  = 4,
	TypeStick    = 5,
};
