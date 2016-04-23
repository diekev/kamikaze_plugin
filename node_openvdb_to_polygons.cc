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

#include <kamikaze/mesh.h>
#include <kamikaze/nodes.h>
#include <kamikaze/paramfactory.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>

#include "levelset.h"

static constexpr auto NODE_NAME = "OpenVDB To Polygons";

class NodeToPolygons : public Node {
	float m_iso_value = 0.0f;
	float m_adaptivity = 0.0f;

public:
	NodeToPolygons();
	~NodeToPolygons() = default;

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeToPolygons::NodeToPolygons()
    : Node(NODE_NAME)
{
	addInput("VDB");
	addOutput("Mesh");
}

void NodeToPolygons::setUIParams(ParamCallback *cb)
{
	float_param(cb, "Iso Value", &m_iso_value, -1.0f, 1.0f, 0.0f);
	param_tooltip(cb, "Crossing point value considered as the surface.");

	float_param(cb, "Adaptivity", &m_adaptivity, 0.0f, 1.0f, 0.0f);
	param_tooltip(cb, "Determine how closely the isosurface is matched by the resulting mesh..");
}

void NodeToPolygons::process()
{
	auto prim = getInputPrimitive("VDB");

	if (!prim) {
		setOutputPrimitive("Mesh", nullptr);
		return;
	}

	auto *ls = static_cast<LevelSet *>(prim);

	auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(ls->getGridPtr());

	openvdb::tools::VolumeToMesh mesher(m_iso_value, m_adaptivity);
	mesher(*grid);

	Mesh *mesh = new Mesh;

	mesh->verts().reserve(mesher.pointListSize());

	for (size_t i = 0, ie = mesh->verts().size(); i < ie; ++i) {
		const auto &point = mesher.pointList()[i];
		mesh->verts()[i] = { point[0], point[1], point[2] };
	}

	auto &polygon_pool_list = mesher.polygonPoolList();

	/* Preallocate primitive lists */
	size_t num_quads = 0, num_triangles = 0;
	for (size_t n = 0, ie = mesher.polygonPoolListSize(); n < ie; ++n) {
		auto &polygons = polygon_pool_list[n];

		num_triangles += polygons.numTriangles();
		num_quads += polygons.numQuads();
	}

	mesh->quads().reserve(num_quads);
	mesh->tris().reserve(num_triangles);

	size_t quad_idx = 0, tri_idx = 0;
    for (size_t n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
        auto &polygons = polygon_pool_list[n];

        for (size_t i = 0, I = polygons.numQuads(); i < I; ++i) {
			const auto &quad = polygons.quad(i);
            mesh->quads()[quad_idx++] = { quad[0], quad[1], quad[2], quad[3] };
        }

        for (size_t i = 0, I = polygons.numTriangles(); i < I; ++i) {
			const auto &tri = polygons.triangle(i);
            mesh->tris()[tri_idx++] = { tri[0], tri[1], tri[2] };
        }
    }

	mesh->update();

	setOutputPrimitive("Mesh", mesh);
}

static Node *new_to_polygons_node()
{
	return new NodeToPolygons;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_to_polygons_node);
}

}
