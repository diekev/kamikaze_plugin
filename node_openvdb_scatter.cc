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
#include "node_openvdb.h"

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/PointScatter.h>

#include <random>

#include "util_openvdb_process.h"
#include "volumebase.h"

#include "util_string.h"

static constexpr auto NODE_NAME = "OpenVDB Scatter";

class NodeOpenVDBScatter : public VDBNode {
public:
	NodeOpenVDBScatter();

	bool update_properties() override;

	void process() override;

	void process_impl();
};

NodeOpenVDBScatter::NodeOpenVDBScatter()
    : VDBNode(NODE_NAME)
{
	addInput("VDB");
	addOutput("Points");

	add_prop("Keep input VDB grids", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("The output will contain the input VDB grids.");

	add_prop("Random Seed", property_type::prop_int);
	set_prop_min_max(0, 1000);

	add_prop("Spread", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Defines how far each point may be displaced from the center "
	                 "of its voxel or tile. A value of zero means that the point is "
	                 "placed exactly at the center. A value of one means that the "
	                 "point can be placed randomly anywhere inside the voxel or tile.");

	EnumProperty mode_enum;
	mode_enum.insert("Fixed", 0);
	mode_enum.insert("Density", 1);
	mode_enum.insert("Points Per Voxel", 2);

	add_prop("Mode", property_type::prop_enum);
	set_prop_enum_values(mode_enum);
	set_prop_tooltip("Specify how to scatter points inside the volume:\n"
	                 "Fixed: total number of points\n"
                     "Density: number of points per unit volume\n"
                     "Points Per Voxel: number of points per voxel\n");

	add_prop("Count", property_type::prop_int);
	set_prop_min_max(0.0f, 10000.0f);
	set_prop_default_value_int(5000);

	add_prop("Point Density", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(1.0f);

	add_prop("Scale Density", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("Use voxel values as local multipliers for the point density.");

	add_prop("Point Count", property_type::prop_float);
	set_prop_min_max(0.0f, 100.0f);
	set_prop_default_value_float(8.0f);

	add_prop("Scatter Interior", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("Toggle to scatter points in the interior region of a level set. "
	                 "(Instead of the narrow band region used by default.)");

	add_prop("Verbose", property_type::prop_bool);
	set_prop_default_value_bool(false);
}

bool NodeOpenVDBScatter::update_properties()
{
	const auto mode = eval_int("Mode");

    set_prop_visible("Count",         (mode == 0));
    set_prop_visible("Point Density", (mode == 1));
    set_prop_visible("Scale Density", (mode == 1));
    set_prop_visible("Point Count",   (mode == 2));

	return true;
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
	try {
		process_impl();
	}
	catch (const std::exception &e) {
		this->add_warning(e.what());
	}
}

void NodeOpenVDBScatter::process_impl()
{
	const auto seed = eval_int("Random Seed");
	const auto mode = eval_enum("Mode");
	const auto count = static_cast<openvdb::Index64>(eval_int("Count"));
	const auto count_per_voxel = eval_float("Point Count");
	const auto density = eval_float("Point Density");
	const auto interior = eval_bool("Scatter Interior");
	const auto multiply = eval_bool("Scale Density");
	const auto verbose = eval_bool("Verbose");
	const auto spread = eval_float("Spread");
	const auto keep = eval_bool("Keep input VDB grids");

	std::vector<Primitive *> primitives;
	primitives.reserve(m_collection->primitives().size());

	using RandGen = std::mt19937;
	RandGen rng(seed + 198653421);

	openvdb::util::NullInterrupter boss;

	std::vector<std::string> empty_grids;
	std::vector<Primitive *> to_destroy;

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);

		auto points_prim = new PrimPoints;
		primitives.push_back(points_prim);

		PointAccessor points(points_prim->points());

		auto grid = vdb_prim->getGridPtr();
		auto gridName = grid->getName();

		if (grid->empty()) {
            empty_grids.push_back(gridName);
            continue;
        }

		to_destroy.push_back(prim);

		switch (mode) {
			default:
			case 0:
			{
				openvdb::tools::UniformPointScatter<PointAccessor, RandGen>
	                scatter(points, count, rng, spread, &boss);

				if (interior && is_level_set(vdb_prim)) {
					process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}
				else {
					process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}

				if (verbose) {
					scatter.print(gridName);
				}

				break;
			}
			case 1:
			{
				if (multiply) {
					openvdb::tools::NonUniformPointScatter<PointAccessor, RandGen>
		                scatter(points, density, rng, spread, &boss);

					if (interior && is_level_set(vdb_prim)) {
						process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
					}
					else {
						process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
					}

					if (verbose) {
						scatter.print(gridName);
					}
				}
				else {
					openvdb::tools::UniformPointScatter<PointAccessor, RandGen>
		                scatter(points, density, rng, spread, &boss);

					if (interior && is_level_set(vdb_prim)) {
						process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
					}
					else {
						process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
					}

					if (verbose) {
						scatter.print(gridName);
					}
				}

				break;
			}
			case 2:
			{
				openvdb::tools::DenseUniformPointScatter<PointAccessor, RandGen>
	                scatter(points, count_per_voxel, rng, spread, &boss);

				if (interior && is_level_set(vdb_prim)) {
					process_sdf_interior(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}
				else {
					process_grid_real(vdb_prim->getGrid(), vdb_prim->storage(), scatter);
				}

				if (verbose) {
					scatter.print(gridName);
				}

				break;
			}
		}

		points_prim->tagUpdate();
	}

	if (!empty_grids.empty()) {
		std::string s = "The following grids were empty: " + join(empty_grids, ", ");
        this->add_warning(s.c_str());
	}

	if (!keep) {
		m_collection->destroy(to_destroy);
	}

	for (auto &prim : primitives) {
		m_collection->add(prim);
	}
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBScatter);
}

}
