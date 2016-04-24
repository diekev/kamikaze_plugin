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
	int mode;
	float tolerance;

public:
	NodePrune();

	void setUIParams(ParamCallback *cb) override;
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
}

void NodePrune::setUIParams(ParamCallback *cb)
{
	const char *mode_items[] = {
	    "Value", "Inactive", "Level Set", nullptr
	};

	enum_param(cb, "Mode", &mode, mode_items, 0);

	float_param(cb, "Tolerance", &tolerance, 0.0f, 10.0f, 0.0f);
}

void NodePrune::process()
{
	auto prim = getInputPrimitive("VDB");

	if (!prim) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	auto vdb_prim = static_cast<VDBVolume *>(prim);

	PruneOp op(mode, tolerance);

	process_typed_grid(vdb_prim->getGrid(), vdb_prim->storage(), op);

	setOutputPrimitive("VDB", vdb_prim);
}

static Node *new_prune_node()
{
	return new NodePrune;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_prune_node);
}

}
