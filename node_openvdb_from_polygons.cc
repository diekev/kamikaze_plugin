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

#include <kamikaze/mesh.h>
#include <kamikaze/utils_glm.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>

#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB From Polygons";

class NodeFromPolygons : public VDBNode {
public:
	NodeFromPolygons();
	~NodeFromPolygons() = default;

	void process() override;
};


NodeFromPolygons::NodeFromPolygons()
    : VDBNode(NODE_NAME)
{
	addInput("Mesh");
	addOutput("VDB");

	add_prop("Voxel Size", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(0.1f);
	set_prop_tooltip("Uniform voxel size in world units of the generated level set.");

	add_prop("Interior Band", property_type::prop_int);
	set_prop_min_max(1, 10);
	set_prop_default_value_int(3);
	set_prop_tooltip("Interior width of the generated level set.");

	add_prop("Exterior Band", property_type::prop_int);
	set_prop_min_max(1, 10);
	set_prop_default_value_int(3);
	set_prop_tooltip("Exterior width of the generated level set.");
}

void NodeFromPolygons::process()
{
	const auto voxel_size = eval_float("Voxel Size");
	const auto int_band = eval_int("Interior Band");
	const auto ext_band = eval_int("Exterior Band");

	std::vector<Primitive *> converted_prims;

	auto transform = openvdb::math::Transform::createLinearTransform(voxel_size);

	std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec4I> faces;

	for (auto prim : primitive_iterator(m_collection, Mesh::id)) {
		converted_prims.push_back(prim);

		auto mesh = static_cast<Mesh *>(prim);
		const auto mpoints = mesh->points();
		const auto polys = mesh->polys();

		points.clear();
		points.reserve(mpoints->size());
		faces.clear();
		faces.reserve(polys->size());

		openvdb::Vec3s point;
		for (auto n = 0ul, N = mpoints->size(); n < N; ++n) {
			const auto &vert = mesh->matrix() * (*mpoints)[n];
			point = transform->worldToIndex({ vert[0], vert[1], vert[2] });
			points.push_back(point);
		}

		for (auto n = 0ul, N = polys->size(); n < N; ++n) {
			const auto &quad = (*polys)[n];
			faces.emplace_back(quad[0], quad[1], quad[2], quad[3]);
		}

		openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> adapt(points, faces);

		auto grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
		                adapt, *transform, int_band, ext_band, 0, nullptr);

		build_vdb_prim(m_collection, grid);
	}

	/* Remove the converted meshes from the collection. */
	m_collection->destroy(converted_prims);
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeFromPolygons);
}

}
