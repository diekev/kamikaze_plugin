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

#include <openvdb/tools/GridTransformer.h>

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Resample";

enum {
	NEAREST = 0,
	LINEAR,
	QUADRATIC,
};

class NodeResample : public Node {
	float m_voxel_size = 0.1f;
	float m_voxel_scale = 1.0f;
	int m_order = 0;
	float m_translate[3];
	float m_rotate[3];
	float m_scale[3];
	float m_pivot[3];

public:
	NodeResample();
	~NodeResample() = default;

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeResample::NodeResample()
    : Node(NODE_NAME)
{
	addInput("VDB");
	addOutput("VDB");
}

void NodeResample::setUIParams(ParamCallback *cb)
{
	const char *order_items[] = {
	    "Nearest",
	    "Linear",
	    "Quadratic",
	    nullptr
	};

	enum_param(cb, "Interpolation", &m_order, order_items, m_order);

	const char *eval_items[] = {
	    "Explicit",
	    "Voxel Size",
	    "Scale Voxel",
	    nullptr
	};

	enum_param(cb, "Interpolation", &m_order, eval_items, m_order);

	float_param(cb, "Voxel Size", &m_voxel_size, 0.001f, 10.0f, m_voxel_size);
	float_param(cb, "Voxel Scale", &m_voxel_scale, 0.001f, 10.0f, m_voxel_size);

	xyz_param(cb, "Translate", m_translate);
	xyz_param(cb, "Rotate", m_rotate);
	xyz_param(cb, "Scale", m_scale);
	xyz_param(cb, "Scale", m_pivot);
}

struct LevelSetRebuildOp {
	const float voxel_size;
	openvdb::GridBase::Ptr output;

	explicit LevelSetRebuildOp(const float vsize)
	    : voxel_size(vsize)
	{}

	template <typename GridType>
	void operator()(typename GridType::Ptr grid)
	{
		using namespace openvdb;

		typedef typename GridType::ValueType ValueType;
		typedef math::Transform Transform;

		Transform::Ptr xform = Transform::createLinearTransform(voxel_size);

		const float halfwidth = grid->background() * (1.0f / grid->transform().voxelSize()[0]);
		util::NullInterrupter interrupt;

		output = tools::doLevelSetRebuild(*grid, zeroVal<ValueType>(),
		                                  halfwidth, halfwidth,
		                                  xform.get(), &interrupt);
	}
};

struct ResampleGridOp {
	const float voxel_size;
	openvdb::GridBase::Ptr output;
	const int order;

	explicit ResampleGridOp(openvdb::GridBase::Ptr out, const float vsize, const int ord)
	    : voxel_size(vsize)
	    , output(out)
	    , order(ord)
	{}

	template <typename GridType>
	void operator()(typename GridType::Ptr grid)
	{
		using namespace openvdb;

		typedef typename GridType::ValueType ValueType;
		typedef math::Transform Transform;

		Transform::Ptr xform = Transform::createLinearTransform(voxel_size);

		auto outgrid = gridPtrCast<GridType>(output);
		outgrid->setTransform(xform);
		outgrid->setName(grid->getName());
		outgrid->setGridClass(grid->getGridClass());

		switch (order) {
			case NEAREST:
				tools::resampleToMatch<openvdb::tools::PointSampler>(*grid, *outgrid);
				break;
			case LINEAR:
				tools::resampleToMatch<openvdb::tools::BoxSampler>(*grid, *outgrid);
				break;
			case QUADRATIC:
				tools::resampleToMatch<openvdb::tools::QuadraticSampler>(*grid, *outgrid);
				break;
		}
	}
};

void NodeResample::process()
{
	using namespace openvdb;

	auto prim = getInputPrimitive("VDB");

	if (!prim) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	auto vdb_prim = static_cast<VDBVolume *>(prim);
	auto grid = vdb_prim->getGridPtr();

	auto outgrid = grid->copyGrid(CP_NEW);

	if (is_level_set(vdb_prim)) {
		LevelSetRebuildOp op(m_voxel_size);

		process_grid_real(grid, vdb_prim->storage(), op);
		outgrid = op.output;
	}
	else {
		ResampleGridOp op(outgrid, m_voxel_size, m_order);
		process_typed_grid(grid, vdb_prim->storage(), op);
	}

	vdb_prim->setGrid(outgrid);

	setOutputPrimitive("VDB", vdb_prim);
}

static Node *new_resample_node()
{
	return new NodeResample;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_resample_node);
}

}
