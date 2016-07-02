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
public:
	NodeResample();
	~NodeResample() = default;

	void process() override;
};

NodeResample::NodeResample()
    : Node(NODE_NAME)
{
	addInput("VDB");
	addOutput("VDB");

	EnumProperty order_enum;
	order_enum.insert("Nearest",   NEAREST);
	order_enum.insert("Linear",    LINEAR);
	order_enum.insert("Quadratic", QUADRATIC);

	add_prop("Interpolation", property_type::prop_enum);
	set_prop_enum_values(order_enum);

	EnumProperty eval_enum;
	eval_enum.insert("Explicit", 0);
	eval_enum.insert("Voxel Size", 1);
	eval_enum.insert("Scale Voxel", 2);

	add_prop("Evaluation", property_type::prop_enum);
	set_prop_enum_values(eval_enum);

	add_prop("Voxel Size", property_type::prop_float);
	set_prop_min_max(0.01f, 10.0f);
	set_prop_default_value_float(0.1f);
	set_prop_tooltip("Uniform voxel size in world units of the generated level set.");

	add_prop("Voxel Scale", property_type::prop_float);
	set_prop_min_max(0.01f, 10.0f);
	set_prop_default_value_float(1.0f);

	add_prop("Translate", property_type::prop_vec3);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});

	add_prop("Rotate", property_type::prop_vec3);
	set_prop_min_max(0.0f, 360.0f);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});

	add_prop("Scale", property_type::prop_vec3);
	set_prop_default_value_vec3(glm::vec3{1.0f, 1.0f, 1.0f});

	add_prop("Pivot", property_type::prop_vec3);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});
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

	const auto voxel_size = eval_float("Voxel Size");
//	const auto m_voxel_scale = eval_float("Voxel Scale");
	const auto order = eval_int("Interpolation");
//	const auto translate = eval_vec3("Translate");
//	const auto rotate = eval_vec3("Rotate");
//	const auto scale = eval_vec3("Scale");
//	const auto pivot = eval_vec3("Pivot");

	auto vdb_prim = static_cast<VDBVolume *>(prim);
	auto grid = vdb_prim->getGridPtr();

	auto outgrid = grid->copyGrid(CP_NEW);

	if (is_level_set(vdb_prim)) {
		LevelSetRebuildOp op(voxel_size);

		process_grid_real(grid, vdb_prim->storage(), op);
		outgrid = op.output;
	}
	else {
		ResampleGridOp op(outgrid, voxel_size, order);
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
