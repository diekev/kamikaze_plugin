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

#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB MetaData";

#define set_class_str "Set Class"
#define class_str "Class"

#define set_creator_str "Set Creator"
#define creator_str "Creator"

#define set_world_str "Set World"
#define world_str "Transform Values"

#define set_vectype_str "Set Vector Type"
#define vectype_str "Vector Type"

#define set_float16_str "Set 16-Bit Floats"
#define float16_str "Write 16-Bit Floats"

class NodeOpenVDBMetaData : public Node {
public:
	NodeOpenVDBMetaData();

	bool update_properties() override;

	void process() override;
};


NodeOpenVDBMetaData::NodeOpenVDBMetaData()
    : Node(NODE_NAME)
{
	addInput("input");
	addOutput("output");

	add_prop(set_class_str, property_type::prop_bool);
	set_prop_default_value_bool(false);

	{
		EnumProperty type_enum;

		for (int n = 0; n < openvdb::NUM_GRID_CLASSES; ++n) {
			openvdb::GridClass gridclass = static_cast<openvdb::GridClass>(n);
			type_enum.insert(openvdb::GridBase::gridClassToMenuName(gridclass), n);
		}

		add_prop(class_str, property_type::prop_enum);
		set_prop_enum_values(type_enum);
		set_prop_tooltip("Specify how the grid's values should be interpreted.");
	}

	add_prop(set_creator_str, property_type::prop_bool);
	set_prop_default_value_bool(false);

	add_prop(creator_str, property_type::prop_string);
	set_prop_tooltip("Specify who created the grid(s).");

	add_prop(set_world_str, property_type::prop_bool);
	set_prop_default_value_bool(false);

	add_prop(world_str, property_type::prop_bool);
	set_prop_default_value_bool(true);
	set_prop_tooltip("For vector-valued grids, specify whether voxel values\n"
	                 "are in world space and should be affected by transforms\n"
	                 "or in local space and should not be transformed.");

	add_prop(set_vectype_str, property_type::prop_bool);
	set_prop_default_value_bool(false);

	{
		std::string help =
		        "For vector-valued grids, specify how voxel values are affected by transforms:\n";

		EnumProperty type_enum;

		for (int n = 0; n < openvdb::NUM_VEC_TYPES; ++n) {
			openvdb::VecType vectype = static_cast<openvdb::VecType>(n);
			type_enum.insert(openvdb::GridBase::vecTypeToString(vectype), n);

			help += "\n" + openvdb::GridBase::vecTypeExamples(vectype) + "\n    "
			        + openvdb::GridBase::vecTypeDescription(vectype) + ".";
		}

		add_prop(vectype_str, property_type::prop_enum);
		set_prop_enum_values(type_enum);
		set_prop_tooltip(help);
	}

	add_prop(set_float16_str, property_type::prop_bool);
	set_prop_default_value_bool(false);

	add_prop(float16_str, property_type::prop_bool);
	set_prop_default_value_bool(true);
	set_prop_tooltip("When saving the grid to a file, write floating-point\n"
	                 "scalar or vector voxel values as 16-bit half floats.");
}

bool NodeOpenVDBMetaData::update_properties()
{
	set_prop_visible(class_str, eval_bool(set_class_str));
	set_prop_visible(creator_str, eval_bool(set_creator_str));
	set_prop_visible(world_str, eval_bool(set_world_str));
	set_prop_visible(vectype_str, eval_bool(set_vectype_str));
	set_prop_visible(float16_str, eval_bool(set_float16_str));
	return true;
}

void NodeOpenVDBMetaData::process()
{
	try {
		/* Get UI parameter values. */
		const auto set_class = eval_bool(set_class_str);
		const auto gridclass = static_cast<openvdb::GridClass>(eval_enum(class_str));

		const auto set_creator = eval_bool(set_creator_str);
		const auto creator = eval_string(creator_str);

		const auto set_float16 = eval_bool(set_float16_str);
		const auto float16 = eval_bool(float16_str);

		const auto set_vectype = eval_bool(set_vectype_str);
		const auto vectype = static_cast<openvdb::VecType>(eval_enum(vectype_str));

		const auto set_world = eval_bool(set_world_str);
		const auto world = eval_bool(world_str);

		/* Set metadatas to the volume grids in the collection. */
		for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
			auto vdb = static_cast<VDBVolume *>(prim);
			auto &grid = vdb->getGrid();

			if (set_class) {
				grid.setGridClass(gridclass);

				/* TODO: update viewport visualisation settings. */
			}

			if (set_creator) {
				grid.setCreator(creator);
			}

			if (set_float16) {
				grid.setSaveFloatAsHalf(float16);
			}

			if (set_vectype) {
				grid.setVectorType(vectype);
			}

			if (set_world) {
				grid.setIsInWorldSpace(world);
			}
		}
	}
	catch (const std::exception &e) {
		std::cerr << "NodeOpenVDBMetaData::process: " << e.what() << '\n';
	}
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBMetaData);
}

}
