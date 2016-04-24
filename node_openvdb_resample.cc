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

class NodeResample : public Node {
	float voxel_size = 0.1f;

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
	float_param(cb, "Voxel Size", &voxel_size, 0.1f, 10.0f, voxel_size);
}

struct LevelSetRebuildOp {
	const float voxel_size;

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
		typename GridType::Ptr output;

		const float halfwidth = grid->background() * (1.0f / grid->transform().voxelSize()[0]);
		util::NullInterrupter interrupt;

		output = tools::doLevelSetRebuild(*grid, zeroVal<ValueType>(),
		                                  halfwidth, halfwidth,
		                                  xform.get(), &interrupt);

		grid.swap(output);
	}
};

struct ResampleGridOp {
	const float voxel_size;

	explicit ResampleGridOp(const float vsize)
	    : voxel_size(vsize)
	{}

	template <typename GridType>
	void operator()(typename GridType::Ptr grid)
	{
		using namespace openvdb;

		typedef typename GridType::ValueType ValueType;
		typedef math::Transform Transform;

		Transform::Ptr xform = Transform::createLinearTransform(voxel_size);
		typename GridType::Ptr output;

		output = GridType::create(grid->background());
		output->setTransform(xform);
		output->setName(grid->getName());
		output->setGridClass(grid->getGridClass());

		tools::resampleToMatch<openvdb::tools::PointSampler>(*grid, *output);

		grid.swap(output);
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

	if (is_level_set(vdb_prim)) {
		LevelSetRebuildOp op(voxel_size);
		process_grid_real(grid, vdb_prim->storage(), op);
	}
	else {
		ResampleGridOp op(voxel_size);
		process_typed_grid(grid, vdb_prim->storage(), op);
	}

	vdb_prim->setGrid(grid);

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
