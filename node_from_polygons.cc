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

#include "node_from_polygons.h"

#include <kamikaze/mesh.h>
#include <kamikaze/paramfactory.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>

#include "levelset.h"

static constexpr auto NODE_NAME = "VDB From Polygons";

NodeFromPolygons::NodeFromPolygons()
    : Node(NODE_NAME)
{
	addInput("Mesh");
	addOutput("VDB");
}

void NodeFromPolygons::setUIParams(ParamCallback *cb)
{
	float_param(cb, "Voxel Size", &m_voxel_size, 0.01f, 10.0f, 0.1f);
	param_tooltip(cb, "Voxel size of the generated level set.");

	int_param(cb, "Interior Band", &m_int_band, 1, 10, 3);
	param_tooltip(cb, "Interior width of the generated level set.");

	int_param(cb, "Exterior Band", &m_ext_band, 1, 10, 3);
	param_tooltip(cb, "Exterior width of the generated level set.");
}

void NodeFromPolygons::process()
{
	auto prim = getInputPrimitive("Mesh");

	if (!prim) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	Mesh *mesh = static_cast<Mesh *>(prim);

	std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec4I> faces;

    {
        points.reserve(mesh->verts().size());
        faces.reserve(mesh->quads().size() + mesh->tris().size());

		for (auto &vert : mesh->verts()) {
			points.emplace_back(vert[0], vert[1], vert[2]);
		}

		for (auto &quad : mesh->quads()) {
			faces.emplace_back(quad[0], quad[1], quad[2], quad[3]);
		}

		for (auto &tri : mesh->tris()) {
			faces.emplace_back(tri[0], tri[1], tri[2], openvdb::util::INVALID_IDX);
		}
    }

	auto transform = openvdb::math::Transform::createLinearTransform(m_voxel_size);

	openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> adapt(points, faces);

	auto grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
	                adapt, *transform, m_int_band, m_ext_band, 0, nullptr);

	auto output_prim = new LevelSet(grid);

	setOutputPrimitive("VDB", output_prim);
}

static Node *new_node()
{
	return new NodeFromPolygons;
}

void NodeFromPolygons::registerSelf(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_node);
}
