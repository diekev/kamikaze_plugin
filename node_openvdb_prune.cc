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
#include <openvdb/tools/Prune.h>

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Prune";

enum {
	VALUE = 0,
	INACTIVE = 1,
	LEVEL_SET = 2,
};

class NodePrune : public Node {

public:
	NodePrune();

	void process() override;
};

class PruneOp {
	const int mode;
	const float tolerance;

public:
	PruneOp(int m, float t)
	    : mode(m)
	    , tolerance(t)
	{}

	template<typename GridType>
    void operator()(GridType &grid) const
    {
        using ValueT = typename GridType::ValueType;

        switch (mode) {
			case VALUE:
				openvdb::tools::prune(grid.tree(), ValueT(openvdb::zeroVal<ValueT>() + tolerance));
				break;
			case INACTIVE:
				openvdb::tools::pruneInactive(grid.tree());
				break;
			case LEVEL_SET:
				openvdb::tools::pruneLevelSet(grid.tree());
				break;
		}
    }
};

NodePrune::NodePrune()
    : Node(NODE_NAME)
{
	addInput("VDB");
	addOutput("VDB");

	EnumProperty mode_enum;
	mode_enum.insert("Value",     VALUE);
	mode_enum.insert("Inactive",  INACTIVE);
	mode_enum.insert("Level Set", LEVEL_SET);

	add_prop("Mode", property_type::prop_enum);
	set_prop_enum_values(mode_enum);

	add_prop("Tolerance", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
}

void NodePrune::process()
{
	const auto mode = eval_int("Mode");
	const auto tolerance = eval_float("Tolerance");

	PruneOp op(mode, tolerance);

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);
		process_typed_grid(vdb_prim->getGrid(), vdb_prim->storage(), op);
	}
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodePrune);
}

}
