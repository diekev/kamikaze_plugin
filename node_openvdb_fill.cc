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

static constexpr auto NODE_NAME = "OpenVDB Fill";

enum {
	INDEX = 0,
	WORLD = 1,
};

class NodeFill : public Node {
	int mode = 0;
	float value = 0.0f;
	 // coords
	float min[3] = { 0.0f, 0.0f, 0.0f };
	float max[3] = { 1.0f, 1.0f, 1.0f };
	bool activate = false;

public:
	NodeFill();

	void setUIParams(ParamCallback *cb) override;
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
}

void NodeFill::setUIParams(ParamCallback *cb)
{
	const char *mode_items[] = {
	    "Index", "World", nullptr
	};

	enum_param(cb, "Mode", &mode, mode_items, mode);
	param_tooltip(cb,
	              "Index - Input coordinates are set in index space\n"
	              "World - Input coordinates are set in world space");

	xyz_param(cb, "Min", min, 0.0f, 1000.0f);
	xyz_param(cb, "Max", max, 0.0f, 1000.0f);

	float_param(cb, "Value", &value, 0.0f, 10.0f, value);
	param_tooltip(cb, "Value to fill the voxels with");

	bool_param(cb, "Set Active", &activate, activate);
	param_tooltip(cb, "Mark voxels in the filled region as active");
}

void NodeFill::process()
{
	auto prim = getInputPrimitive("VDB");

	if (!prim) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	auto vdb_prim = static_cast<VDBVolume *>(prim);

	std::unique_ptr<FillOp> op;

	switch (mode) {
		case INDEX:
		{
			openvdb::math::Coord co_min((int)min[0], (int)min[1], (int)min[2]);
			openvdb::math::Coord co_max((int)max[0], (int)min[1], (int)min[2]);

			op.reset(new FillOp(openvdb::math::CoordBBox(co_min, co_max), value, activate));
			break;
		}
		case WORLD:
			op.reset(new FillOp(openvdb::BBoxd(min, max), value, activate));
			break;
	}

	auto grid = vdb_prim->getGridPtr();
	process_typed_grid(*grid, vdb_prim->storage(), *op);

	vdb_prim->setGrid(grid);

	setOutputPrimitive("VDB", vdb_prim);
}

static Node *new_fill_node()
{
	return new NodeFill;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_fill_node);
}

}
