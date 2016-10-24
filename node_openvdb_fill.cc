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

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Fill";

enum {
	INDEX = 0,
	WORLD = 1,
};

class NodeFill : public Node {
public:
	NodeFill();

	void process() override;
};

class FillOp {
	const openvdb::math::CoordBBox indexBBox;
    const openvdb::BBoxd worldBBox;
	const float value;
	const bool active;

public:
	FillOp(const openvdb::math::CoordBBox &b, const float val, bool on)
	    : indexBBox(b)
	    , value(val)
	    , active(on)
    {}

    FillOp(const openvdb::BBoxd &b, const float val, bool on)
	    : worldBBox(b)
	    , value(val)
	    , active(on)
    {}

	template<typename GridType>
    void operator()(GridType &grid) const
    {
        using ValueT = typename GridType::ValueType;

		auto bbox = indexBBox;

        if (worldBBox) {
            openvdb::math::Vec3d imin, imax;
            openvdb::math::calculateBounds(grid.constTransform(),
               worldBBox.min(), worldBBox.max(), imin, imax);

            bbox.reset(openvdb::Coord::floor(imin), openvdb::Coord::ceil(imax));
        }

        grid.fill(bbox, ValueT(value), active);
    }
};

NodeFill::NodeFill()
    : Node(NODE_NAME)
{
	addInput("VDB");
	addOutput("VDB");

	EnumProperty mode_enum;
	mode_enum.insert("Index", 0);
	mode_enum.insert("World", 1);

	add_prop("Mode", property_type::prop_enum);
	set_prop_enum_values(mode_enum);
	set_prop_tooltip("Index - Input coordinates are set in index space\n"
	                 "World - Input coordinates are set in world space");

	add_prop("Min", property_type::prop_vec3);
	set_prop_min_max(0.0f, 1000.0f);
	set_prop_default_value_vec3(glm::vec3{ 0.0f, 0.0f, 0.0f });

	add_prop("Max", property_type::prop_vec3);
	set_prop_min_max(0.0f, 1000.0f);
	set_prop_default_value_vec3(glm::vec3{ 1.0f, 1.0f, 1.0f });

	add_prop("Value", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_tooltip("Value to fill the voxels with");

	add_prop("Set Active", property_type::prop_bool);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_tooltip("Mark voxels in the filled region as active");
}

void NodeFill::process()
{
	const auto mode = eval_int("Mode");
	const auto value = eval_float("Value");
	const auto min = eval_vec3("Min");
	const auto max = eval_vec3("Max");
	const auto activate = eval_bool("Set Active");

	std::unique_ptr<FillOp> op;

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);

		switch (mode) {
			case INDEX:
			{
				openvdb::math::Coord co_min((int)min[0], (int)min[1], (int)min[2]);
				openvdb::math::Coord co_max((int)max[0], (int)min[1], (int)min[2]);

				op.reset(new FillOp(openvdb::math::CoordBBox(co_min, co_max), value, activate));
				break;
			}
			case WORLD:
				op.reset(new FillOp(openvdb::BBoxd(&min[0], &max[0]), value, activate));
				break;
		}

		auto grid = vdb_prim->getGridPtr();
		process_typed_grid(*grid, vdb_prim->storage(), *op);

		vdb_prim->setGrid(grid);
	}
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeFill);
}

}
