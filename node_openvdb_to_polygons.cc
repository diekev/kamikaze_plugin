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

#include "volumebase.h"

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

	auto vdb_prim = static_cast<VDBVolume *>(prim);

	if (!is_level_set(vdb_prim)) {
		setOutputPrimitive("Mesh", nullptr);
		return;
	}

	auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(vdb_prim->getGridPtr());

	openvdb::tools::VolumeToMesh mesher(m_iso_value, m_adaptivity);
	mesher(*grid);

	Mesh *mesh = new Mesh;

	PointList *points = mesh->points();
	points->reserve(mesher.pointListSize());

	for (size_t i = 0, ie = mesher.pointListSize(); i < ie; ++i) {
		const auto &point = mesher.pointList()[i];
		points->push_back({ point[0], point[1], point[2] });
	}

	auto &polygon_pool_list = mesher.polygonPoolList();

	/* Preallocate primitive lists */
	size_t num_polys = 0;
	for (size_t n = 0, ie = mesher.polygonPoolListSize(); n < ie; ++n) {
		auto &polygons = polygon_pool_list[n];
		num_polys += polygons.numTriangles() + polygons.numQuads();
	}

	PolygonList *polys = mesh->polys();
	polys->reserve(num_polys);

    for (size_t n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
        auto &polygons = polygon_pool_list[n];

        for (size_t i = 0, I = polygons.numQuads(); i < I; ++i) {
			const auto &quad = polygons.quad(i);
            polys->push_back({ quad[0], quad[1], quad[2], quad[3] });
        }

        for (size_t i = 0, I = polygons.numTriangles(); i < I; ++i) {
			const auto &tri = polygons.triangle(i);
            polys->push_back({ tri[0], tri[1], tri[2], std::numeric_limits<unsigned int>::max() });
        }
    }

	mesh->tagUpdate();

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
