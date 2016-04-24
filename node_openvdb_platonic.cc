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

#include <kamikaze/nodes.h>
#include <kamikaze/paramfactory.h>

#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetPlatonic.h>

#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Platonic";

enum {
	PLATONIC_SPHERE = 0,
	PLATONIC_CUBE   = 1,
	PLATONIC_TETRA  = 2,
	PLATONIC_OCTA   = 3,
	PLATONIC_DODE   = 4,
	PLATONIC_ICOSA  = 5,
};

class NodePlatonic : public Node {
	float voxel_size = 0.1f;
	float half_width = 3.0f;
	float radius = 2.0f;
	float center[3] = { 0.0f, 0.0f, 0.0f };
	int type = PLATONIC_SPHERE;

public:
	NodePlatonic();
	~NodePlatonic() = default;

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodePlatonic::NodePlatonic()
    : Node(NODE_NAME)
{
	addOutput("VDB");
}

void NodePlatonic::setUIParams(ParamCallback *cb)
{
	const char *platonic_type[] = {
	    "Sphere", "Cube", "Tetrahedron", "Octahedron", "Dodecahedron",
	    "Icosahedron", nullptr
	};

	enum_param(cb, "Solid Type", &type, platonic_type, type);

	float_param(cb, "Radius", &radius, 0.1f, 10.0f, radius);
	float_param(cb, "Voxel Size", &voxel_size, 0.1f, 10.0f, voxel_size);
	float_param(cb, "Half Width", &half_width, 3.0f, 10.0f, half_width);

	xyz_param(cb, "Center", center);
}

void NodePlatonic::process()
{
	openvdb::FloatGrid::Ptr grid;

	switch (type) {
		default:
		case PLATONIC_SPHERE:
			grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(center), voxel_size, half_width);
			break;
		case PLATONIC_CUBE:
			grid = openvdb::tools::createLevelSetCube<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(center), voxel_size, half_width);
			break;
		case PLATONIC_TETRA:
			grid = openvdb::tools::createLevelSetTetrahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(center), voxel_size, half_width);
			break;
		case PLATONIC_OCTA:
			grid = openvdb::tools::createLevelSetOctahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(center), voxel_size, half_width);
			break;
		case PLATONIC_DODE:
			grid = openvdb::tools::createLevelSetDodecahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(center), voxel_size, half_width);
			break;
		case PLATONIC_ICOSA:
			grid = openvdb::tools::createLevelSetIcosahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(center), voxel_size, half_width);
			break;
	}

	auto prim = new VDBVolume(grid);
	setOutputPrimitive("VDB", prim);
}

static Node *new_platonic_node()
{
	return new NodePlatonic;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_platonic_node);
}

}
