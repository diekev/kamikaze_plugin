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
 * along with this program; if not, write to the Free Software  Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2016 Kévin Dietrich.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

#include "node_platonic.h"

#include <kamikaze/paramfactory.h>

#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetUtil.h>

#include "levelset.h"

static constexpr auto NODE_NAME = "Platonic (VDB)";

NodePlatonic::NodePlatonic()
    : Node(NODE_NAME)
{
	addOutput("Primitive");
}

void NodePlatonic::setUIParams(ParamCallback *cb)
{
	const char *platonic_type[] = {
	    "Sphere", "Cube", nullptr
	};

	enum_param(cb, "Solid Type", &type, platonic_type, type);

	float_param(cb, "Radius", &radius, 0.1f, 10.0f, radius);
	float_param(cb, "Voxel Size", &voxel_size, 0.1f, 10.0f, voxel_size);
	float_param(cb, "Half Width", &half_width, 3.0f, 10.0f, half_width);

	xyz_param(cb, "Center", center);
}

void NodePlatonic::process()
{
	auto prim = input(0)->prim;

	if (!prim) {
		return;
	}

	LevelSet *level_set = dynamic_cast<LevelSet	*>(prim);

	openvdb::FloatGrid::Ptr grid;

	switch (type) {
		default:
		case PLATONIC_SPHERE:
			grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
			              radius, openvdb::Vec3f(center), voxel_size, half_width);
			break;
		case PLATONIC_CUBE:
		{
			auto xform = openvdb::math::Transform::createLinearTransform(voxel_size);

			grid = openvdb::tools::createLevelSetBox<openvdb::FloatGrid>(
			           openvdb::BBoxd(radius * openvdb::Vec3f(-1.0f), radius * openvdb::Vec3f(1.0f)),
			           *xform, half_width);
			break;
		}
	}

	level_set->setGrid(grid);

	output(0)->prim = level_set;
}

static Node *new_resample_node()
{
	return new NodePlatonic;
}

void NodePlatonic::registerSelf(NodeFactory *factory)
{
	factory->registerType(NODE_NAME, new_resample_node);
}
