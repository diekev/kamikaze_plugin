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

#include <kamikaze/prim_points.h>
#include <kamikaze/nodes.h>
#include <kamikaze/paramfactory.h>

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/PointScatter.h>

#include <random>

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Scatter";

class NodeOpenVDBScatter : public Node {
	int m_seed = 0;
	int m_mode = 0;
	float m_count = 8.0f;
	bool m_interior = false;
	bool m_multiply = false;

public:
	NodeOpenVDBScatter();

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeOpenVDBScatter::NodeOpenVDBScatter()
    : Node(NODE_NAME)
{
	addInput("VDB");
	addOutput("Points");
}

void NodeOpenVDBScatter::setUIParams(ParamCallback *cb)
{
	const char *mode_items[] = {
	    "Fixed", "Density", "Points Per Voxel", nullptr
	};

	bool_param(cb, "Scatter Interior", &m_interior, m_interior);
	bool_param(cb, "Multiply Local Density", &m_multiply, m_multiply);

	enum_param(cb, "Mode", &m_mode, mode_items, m_mode);
	param_tooltip(cb, "Specify how to scatter points inside the volume:\n"
	                  "Fixed: total number of points\n"
	                  "Density: number of points per unit volume\n"
	                  "Points Per Voxel: number of points per voxel\n");

	float_param(cb, "Point Count", &m_count, 0, 100, m_count);
	int_param(cb, "Random Seed", &m_seed, 0, 1000, m_seed);
}

/* Simple wrapper class required by openvdb::tools::UniformPointScatter and
 * NonUniformPointScatter */
class PointAccessor {
	PointList *m_points;

public:
    PointAccessor(PointList *list)
	    : m_points(list)
    {}

    void add(const openvdb::Vec3R &pos)
    {
		m_points->push_back({ pos.x(), pos.y(), pos.z() });
    }
};

/* Method to extract the interior mask before scattering points. */
template<typename OpType>
bool process_sdf_interior(const openvdb::GridBase &ref_grid, int storage, OpType &op)
{
    if (storage == GRID_STORAGE_FLOAT) {
        const auto *grid = static_cast<const openvdb::FloatGrid *>(&ref_grid);

		if (grid == nullptr) {
			return false;
		}

        auto maskGrid = openvdb::tools::sdfInteriorMask(*grid);
        op(*maskGrid);

        return true;
    }

	if (storage == GRID_STORAGE_DOUBLE) {
        const auto *grid = static_cast<const openvdb::DoubleGrid *>(&ref_grid);

		if (grid == nullptr) {
			return false;
		}

        auto maskGrid = openvdb::tools::sdfInteriorMask(*grid);
        op(*maskGrid);

        return true;
    }

    return false;
}

void NodeOpenVDBScatter::process()
{
	auto prim = getInputPrimitive("VDB");

	if (!prim) {
		setOutputPrimitive("Points", nullptr);
		return;
	}

	auto vdb_prim = static_cast<VDBVolume *>(prim);

	PrimPoints *points_prim = new PrimPoints;

	PointAccessor points(points_prim->points());

	using RandGen = std::mt19937;
	RandGen rng(m_seed + 198653421);

	switch (m_mode) {
		default:
		case 0:
		{
			openvdb::tools::UniformPointScatter<PointAccessor, RandGen>
                scatter(points, m_count, rng);

			if (m_interior && is_level_set(vdb_prim)) {
				process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
			}
			else {
				process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
			}

			break;
		}
		case 1:
		{
			if (m_multiply) {
				openvdb::tools::NonUniformPointScatter<PointAccessor, RandGen>
	                scatter(points, m_count, rng);

				if (m_interior && is_level_set(vdb_prim)) {
					process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}
				else {
					process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}
			}
			else {
				openvdb::tools::UniformPointScatter<PointAccessor, RandGen>
	                scatter(points, m_count, rng);

				if (m_interior && is_level_set(vdb_prim)) {
					process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}
				else {
					process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}
			}

			break;
		}
		case 2:
		{
			openvdb::tools::DenseUniformPointScatter<PointAccessor, RandGen>
                scatter(points, m_count, rng);

			if (m_interior && is_level_set(vdb_prim)) {
				process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
			}
			else {
				process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
			}

			break;
		}
	}

	points_prim->tag_update();

	setOutputPrimitive("Points", points_prim);
}

static Node *new_scatter_node()
{
	return new NodeOpenVDBScatter;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_scatter_node);
}

}
