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

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Create";

class NodeCreate : public Node {
	QString gridname = "";
	float background = 0.0f;
	float voxel_size = 0.1f;
	int storage = 0;
	int vectype = 0;
	int gridclass = 0;

public:
	NodeCreate();

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeCreate::NodeCreate()
    : Node(NODE_NAME)
{
	addOutput("VDB");
}

void NodeCreate::setUIParams(ParamCallback *cb)
{
	string_param(cb, "Grid Name", &gridname, "VDB Grid");

	const char *storage_items[] = {
	    "Float", "Double", "Bool", "Int32", "Int64", "Vec3I", "Vec3s", "Vec3d",
	    nullptr
	};

	enum_param(cb, "Storage", &storage, storage_items, storage);

	const char *vectype_items[] = {
	    "Invariant", "Covariant", "Covariant Normalize", "Contravariant Relative",
	    "Contravariant Absolute", nullptr
	};

	enum_param(cb, "Vector Type", &vectype, vectype_items, vectype);
	param_tooltip(cb,
	              "The type of a vector determines how transforms are applied to it\n"
	              "Invariant:\n"
	              "    Does not transform (e.g., tuple, uvw, color)\n"
	              "Covariant:\n"
	              "    Apply inverse-transpose transformation: ignores translation, (e.g., gradient/normal)\n"
	              "Covariant Normalize:\n"
	              "    Apply inverse-transpose transformation: ignores translation, vectors are renormalized (e.g., unit normal)\n"
	              "Contravariant Relative:\n"
	              "    Apply \"regular\" transformation: ignores translation (e.g., displacement, velocity, acceleration)\n"
	              "Contravariant Absolute:\n"
	              "    Apply \"regular\" transformation: vector translates (e.g., position)"
	              );

	const char *gridclass_items[] = {
	    "None", "Level Set", "Fog Volume", "Staggered", nullptr
	};

	enum_param(cb, "Grid Class", &gridclass, gridclass_items, gridclass);

	float_param(cb, "Background Value", &background, 0.0f, 10.0f, background);
	param_tooltip(cb, "Unique value returned when accessing a location in space"
	                  " that does not resolve to a voxel or a tile.");

	float_param(cb, "Voxel Size", &voxel_size, 0.01f, 10.0f, voxel_size);
	param_tooltip(cb, "Uniform voxel size in world units.");
}

template<typename T1, typename T2>
auto is_elem(T1 &&a, T2 &&b) -> bool
{
	return a == b;
}

template<typename T1, typename T2, typename... Ts>
auto is_elem(T1 &&a, T2 &&b, Ts &&... t) -> bool
{
	return a == b || is_elem(a, t...);
}

void NodeCreate::process()
{
	int storage_type = storage;

	/* Force a specific type for some grid classes to avoid issues */
	if (gridclass == openvdb::GridClass::GRID_LEVEL_SET) {
		if (!is_elem(storage, GRID_STORAGE_FLOAT, GRID_STORAGE_DOUBLE)) {
			storage_type = GRID_STORAGE_FLOAT;
		}
	}
	else if (gridclass == openvdb::GridClass::GRID_FOG_VOLUME) {
		if (!is_elem(storage, GRID_STORAGE_FLOAT, GRID_STORAGE_DOUBLE)) {
			storage_type = GRID_STORAGE_FLOAT;
		}
	}
	else if (gridclass == openvdb::GridClass::GRID_STAGGERED) {
		if (!is_elem(storage, GRID_STORAGE_VEC3D, GRID_STORAGE_VEC3S, GRID_STORAGE_VEC3I)) {
			storage_type = GRID_STORAGE_VEC3S;
		}
	}

	openvdb::GridBase::Ptr grid;

	switch (storage) {
		case GRID_STORAGE_FLOAT:
			grid = openvdb::FloatGrid::create(background);
			break;
		case GRID_STORAGE_DOUBLE:
			grid = openvdb::DoubleGrid::create(background);
			break;
		case GRID_STORAGE_BOOL:
			grid = openvdb::BoolGrid::create(background);
			break;
		case GRID_STORAGE_INT32:
			grid = openvdb::Int32Grid::create(background);
			break;
		case GRID_STORAGE_INT64:
			grid = openvdb::Int64Grid::create(background);
			break;
		case GRID_STORAGE_VEC3I:
			grid = openvdb::Vec3IGrid::create(openvdb::Vec3I(background));
			break;
		case GRID_STORAGE_VEC3S:
			grid = openvdb::Vec3SGrid::create(openvdb::Vec3s(background));
			break;
		case GRID_STORAGE_VEC3D:
			grid = openvdb::Vec3DGrid::create(openvdb::Vec3d(background));
			break;
	}

	auto transform = openvdb::math::Transform::createLinearTransform(voxel_size);

	grid->setName(gridname.toStdString());
	grid->setTransform(transform);
	grid->setVectorType(static_cast<openvdb::VecType>(vectype));
	grid->setGridClass(static_cast<openvdb::GridClass>(gridclass));

	auto vdb_prim = new VDBVolume(grid);
	setOutputPrimitive("VDB", vdb_prim);
}

static Node *new_create_node()
{
	return new NodeCreate;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_create_node);
}

}
