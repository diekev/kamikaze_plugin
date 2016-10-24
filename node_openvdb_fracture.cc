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

#include "node_openvdb.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <kamikaze/mesh.h>
#include <kamikaze/prim_points.h>
#include <kamikaze/utils_glm.h>

#include <openvdb/tools/LevelSetFracture.h>

#include <random>

#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Fracture";

class NodeOpenVDBFracture : public VDBNode {
public:
	NodeOpenVDBFracture();
	~NodeOpenVDBFracture() = default;

	void process() override;

	void do_process();

	template <typename GridType>
	void do_fracture(std::list<openvdb::GridBase::Ptr> &grids,
	                 PrimitiveCollection *cutters,
	                 PrimitiveCollection *instance_points);

	template <typename GridType>
	void export_fragments(std::list<typename GridType::Ptr> &fracture_tool);
};

enum {
	FRACTURE_VIZ_NONE = 0,
	FRACTURE_VIZ_ALL  = 1,
	FRACTURE_VIZ_NEW  = 2,
};

NodeOpenVDBFracture::NodeOpenVDBFracture()
    : VDBNode(NODE_NAME)
{
	addInput("input");
	addInput("cutters");
	addInput("instance points (optional)");
	addOutput("output");

	add_prop("Separate Cutters", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("Enable if multiple cutter objects are provided."
	                 " This option is only available without instance points.");

	add_prop("Allow Cutter Overlap", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("Allow consecutive cutter instances to fracture previously"
	                 " generated fragments.");

	add_prop("Center Cutter Geometry", property_type::prop_bool);
	set_prop_default_value_bool(true);
	set_prop_tooltip("Pre-center cutter geometry about the origin before instancing.");

	add_prop("Randomize Cutter Rotation", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("Apply a random rotation to each instance point. This "
	                 "option is only available when instance points are provided.");

	add_prop("Random Seed", property_type::prop_int);
	set_prop_default_value_int(1);
	set_prop_min_max(1, 10000);
	set_prop_tooltip("Seed for the random rotation.");

	add_prop("Split Input Fragments into Primitives", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("Split grids with disjoint fragments into multiple grids,"
	                 " one per fragment. In a chain of fracture nodes this "
	                 "operation is typically only applied to the last node.");

	EnumProperty type_enum;
	type_enum.insert("None", FRACTURE_VIZ_NONE);
	type_enum.insert("Pieces", FRACTURE_VIZ_ALL);
	type_enum.insert("New Fragments", FRACTURE_VIZ_NEW);

	add_prop("Visualization", property_type::prop_enum);
	set_prop_enum_values(type_enum);
	set_prop_tooltip("Randomize output primitive colors.");
}

void NodeOpenVDBFracture::process()
{
	try {
		do_process();
    }
	catch (const std::exception &e) {
        std::cerr << "NodeOpenVDBFracture::process: " << e.what() << '\n';
	}
}

void NodeOpenVDBFracture::do_process()
{
    /* Validate input. */
	const auto cutter_collection = getInputCollection("cutters");

    if (cutter_collection == nullptr || cutter_collection->primitives().empty()) {
		std::cerr << "No cutters... returning!\n";
        /* All good, nothing to worry about with no cutting objects! */
        return;
    }

	std::list<openvdb::GridBase::Ptr> grids;
    std::vector<Primitive *> converted_prims;

	std::vector<std::string> non_level_sets, non_linear;

	std::cerr << "Finding grids to process...\n";

	for (Primitive *prim : primitive_iterator(m_collection, VDBVolume::id)) {
		auto volume = static_cast<VDBVolume *>(prim);

		auto grid = volume->getGridPtr();
		const auto grid_class = grid->getGridClass();

		if (grid_class != openvdb::GRID_LEVEL_SET) {
			non_level_sets.push_back(volume->name());
            continue;
        }

		if (!grid->transform().isLinear()) {
            non_linear.push_back(volume->name());
            continue;
        }

        grids.push_back(grid->copyGrid());
        grids.back()->setName(grid->getName());

        converted_prims.push_back(prim);
	}

    if (!non_level_sets.empty()) {
		std::cerr << "The following non level set grids were skipped: '";

		for (const auto &str : non_level_sets) {
			std::cerr << str << ", ";
		}

		std::cerr << "'.\n";
    }

    if (!non_linear.empty()) {
		std::cerr << "The following grids were skipped: '";

		for (const auto &str : non_level_sets) {
			std::cerr << str << ", ";
		}

		std::cerr << "' because they don't have a linear/affine transform.\n";
    }

    if (!grids.empty()) {
		const auto instance_collection = getInputCollection("instance points (optional)");

        if (grids.front()->isType<openvdb::FloatGrid>()) {
            do_fracture<openvdb::FloatGrid>(grids, cutter_collection, instance_collection);
        }
		else if (grids.front()->isType<openvdb::DoubleGrid>()) {
            do_fracture<openvdb::DoubleGrid>(grids, cutter_collection, instance_collection);
        }
		else {
            std::cerr << "Unsupported grid type.\n";
        }

		/* Remove the processed primitives from the collection. */
		m_collection->destroy(converted_prims);
    }
	else {
         std::cerr << "No VDB grids to fracture.\n";
    }
}

template <typename GridType>
static auto get_residuals(std::list<openvdb::GridBase::Ptr> &grids,
                          const openvdb::math::Transform &transform,
                          const typename GridType::ValueType backgroundValue)
{
	std::list<typename GridType::Ptr> residuals;

    std::vector<std::string> bad_transform_list, bad_background_list, bad_type_list;

    for (const auto &grid : grids) {
        auto residual = openvdb::gridPtrCast<GridType>(grid);

        if (!residual) {
			bad_type_list.push_back(residual->getName());
            continue;
        }

		if (residual->transform() != transform) {
            bad_transform_list.push_back(residual->getName());
            continue;
        }

        if (!openvdb::math::isApproxEqual(residual->background(), backgroundValue)) {
            bad_background_list.push_back(residual->getName());
            continue;
        }

        residuals.push_back(residual);
    }

    grids.clear();

    if (!bad_transform_list.empty()) {
		std::cerr << "The following grids were skipped: '";

		for (const auto &str : bad_transform_list) {
			std::cerr << str << ", ";
		}

		std::cerr << "' because they don't match the transform of the first grid.\n";
    }

    if (!bad_background_list.empty()) {
		std::cerr << "The following grids were skipped: '";

		for (const auto &str : bad_background_list) {
			std::cerr << str << ", ";
		}

		std::cerr << "' because they don't match the background value of the first grid.\n";
    }

    if (!bad_type_list.empty()) {
		std::cerr << "The following grids were skipped: '";

		for (const auto &str : bad_type_list) {
			std::cerr << str << ", ";
		}

		std::cerr << "' because they don't have the same data type as the first grid.\n";
    }

	return residuals;
}

static auto get_instance_points(PrimitiveCollection *collection,
                                std::vector<openvdb::Vec3s> &instance_points,
                                std::vector<openvdb::math::Quats> &instance_rotations,
                                bool randomize_rotation,
                                int seed)
{
	for (auto prim : primitive_iterator(collection, PrimPoints::id)) {
		auto point_cloud = static_cast<PrimPoints *>(prim);
		auto particles = point_cloud->points();

		instance_points.resize(particles->size());

		if (randomize_rotation) {
            instance_rotations.resize(instance_points.size());

			std::mt19937 rng(19937 + seed);
			std::uniform_real_distribution<float> dist(0.0f, 1.0f);

            constexpr auto two_pi = 2.0f * static_cast<float>(M_PI);

            glm::mat4 xform;
            glm::quat quat;

			for (auto i = 0ul; i < particles->size(); ++i) {
                auto pos = (*particles)[i];

				/* Generate uniform random rotations by picking random points in
				 * the unit cube and forming the unit quaternion. */

				const auto u  = dist(rng);
				const auto c1 = std::sqrt(1-u);
				const auto c2 = std::sqrt(u);
				const auto s1 = two_pi * dist(rng);
				const auto s2 = two_pi * dist(rng);

				glm::quat orient(c1 * std::sin(s1),
				                 c1 * std::cos(s1),
				                 c2 * std::sin(s2),
				                 c2 * std::cos(s2));

				auto rot = glm::toMat4(orient);
				auto translate = glm::translate(glm::mat4(1.0f), pos);
				xform = rot * translate;
				quat = glm::quat_cast(xform);

                instance_points[i] = openvdb::Vec3s(xform[3][0], xform[3][1], xform[3][2]);
                instance_rotations[i].init(
                    static_cast<float>(quat.x),
                    static_cast<float>(quat.y),
                    static_cast<float>(quat.z),
                    static_cast<float>(quat.w));
            }
        }
        else {
            /* No randomization or valid instance attributes, just use P. */
            for (auto i = 0ul; i < particles->size(); ++i) {
                auto pos = (*particles)[i];
                instance_points[i] = openvdb::Vec3s(pos[0], pos[1], pos[2]);
            }
        }

		/* Only consider the first point cloud. */
		break;
	}
}

template <typename GridType>
void NodeOpenVDBFracture::do_fracture(std::list<openvdb::GridBase::Ptr> &grids,
                                      PrimitiveCollection *cutterGeo,
                                      PrimitiveCollection *pointGeo)
{
	std::cerr << "Begin NodeOpenVDBFracture::do_fracture\n";

	/* Evaluate UI parameters. */
	const auto randomizeRotation = eval_bool("Randomize Cutter Rotation");
    const auto cutterOverlap = eval_bool("Allow Cutter Overlap");
    const auto segmentFragments = eval_bool("Separate Cutters");
	const auto seed = eval_int("Random Seed");

	using ValueType = typename GridType::ValueType;

    auto firstGrid = openvdb::gridPtrCast<GridType>(grids.front());

    if (!firstGrid) {
		std::cerr << "Unsupported grid type.\n";
        return;
    }

    // Get the first grid's transform and background value.
    openvdb::math::Transform::Ptr transform = firstGrid->transformPtr();
    const auto backgroundValue = firstGrid->background();

    std::vector<openvdb::Vec3s> instancePoints;
    std::vector<openvdb::math::Quats> instanceRotations;

    if (pointGeo != NULL) {
        get_instance_points(pointGeo, instancePoints, instanceRotations, randomizeRotation, seed);
    }

	std::cerr << "Getting residuals...\n";

    auto residuals = get_residuals<GridType>(grids, *transform, backgroundValue);

    // Setup fracture tool
    openvdb::tools::LevelSetFracture<GridType> fracture_tool;

    const auto bandWidth = static_cast<float>(backgroundValue / transform->voxelSize()[0]);

	for (auto prim : primitive_iterator(cutterGeo, Mesh::id)) {
		auto mesh = static_cast<Mesh *>(prim);

		const PointList *mpoints = mesh->points();
		const PolygonList *polys = mesh->polys();

		std::vector<openvdb::Vec3s> points;
		std::vector<openvdb::Vec4I> faces;

		std::cerr << "Converting mesh to volume...\n";

		{
			points.reserve(mpoints->size());
			faces.reserve(polys->size());

			openvdb::Vec3s point;
			for (size_t n = 0, N = mpoints->size(); n < N; ++n) {
				const auto &vert = (*mpoints)[n];
				const auto &tmp = mesh->matrix() * vert;
				point = transform->worldToIndex({ tmp[0], tmp[1], tmp[2] });
				points.push_back(point);
			}

			for (size_t n = 0, N = polys->size(); n < N; ++n) {
				const auto &quad = (*polys)[n];
				faces.emplace_back(quad[0], quad[1], quad[2], quad[3]);
			}
		}

		openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> adapt(points, faces);

		auto cutterGrid = openvdb::tools::meshToVolume<GridType>(
		                      adapt, *transform, bandWidth, bandWidth, 0, nullptr);

		if (!cutterGrid || cutterGrid->activeVoxelCount() == 0) {
			return;
		}

		std::cerr << "Fracturing volume...\n";

		fracture_tool.fracture(residuals, *cutterGrid, segmentFragments, &instancePoints,
		                       &instanceRotations, cutterOverlap);

		/* TODO: for now only one cutter allowed. */
		break;
	}

	/* TODO: figure out a way to keep track of the fragments created from a
	 * given objects, so we can suffix them.
	 */

	std::cerr << "Export residual fragments\n";
	/* Export residual fragments. */
	export_fragments<GridType>(residuals);

	std::cerr << "Export new fragments\n";
    /* Export new fragments. */
	export_fragments<GridType>(fracture_tool.fragments());

	std::cerr << "End NodeOpenVDBFracture::do_fracture\n";
}

template <typename GridType>
void NodeOpenVDBFracture::export_fragments(std::list<typename GridType::Ptr> &grids)
{
	for (auto grid : grids) {
		/* TODO: suffix name. */
		build_vdb_prim(m_collection, grid);
    }
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBFracture);
}

}
