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

#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetPlatonic.h>

#include "volumebase.h"

static constexpr auto NOM_OPERATEUR = "OpenVDB Platonic";
static constexpr auto AIDE_OPERATEUR = "";

enum {
	PLATONIC_SPHERE = 0,
	PLATONIC_CUBE   = 1,
	PLATONIC_TETRA  = 2,
	PLATONIC_OCTA   = 3,
	PLATONIC_DODE   = 4,
	PLATONIC_ICOSA  = 5,
};

class NodePlatonic : public OperateurOpenVDB {
public:
	NodePlatonic(Noeud *noeud, const Context &contexte);
	~NodePlatonic() = default;

	const char *nom_sortie(size_t /*index*/) override { return "VDB"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;
};

NodePlatonic::NodePlatonic(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	sorties(1);

	EnumProperty platonic_enum;
	platonic_enum.insert("Sphere",       PLATONIC_SPHERE);
	platonic_enum.insert("Cube",         PLATONIC_CUBE);
	platonic_enum.insert("Tetrahedron",  PLATONIC_TETRA);
	platonic_enum.insert("Octahedron",   PLATONIC_OCTA);
	platonic_enum.insert("Dodecahedron", PLATONIC_DODE);
	platonic_enum.insert("Icosahedron",  PLATONIC_ICOSA);

	add_prop("solid_type", "Solid Type", property_type::prop_enum);
	set_prop_enum_values(platonic_enum);

	add_prop("radius", "Radius", property_type::prop_float);
	set_prop_min_max(0.1f, 10.0f);
	set_prop_default_value_float(2.0f);

	add_prop("voxel_size", "Voxel Size", property_type::prop_float);
	set_prop_min_max(0.01f, 10.0f);
	set_prop_default_value_float(0.1f);
	set_prop_tooltip("Uniform voxel size in world units of the generated level set.");

	add_prop("half_width", "Half Width", property_type::prop_float);
	set_prop_min_max(3.0f, 10.0f);
	set_prop_default_value_float(3.0f);

	add_prop("center", "Center", property_type::prop_vec3);
	set_prop_min_max(-10.0f, 10.0f);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});
}

void NodePlatonic::execute(const Context &contexte, double temps)
{
	const auto voxel_size = eval_float("voxel_size");
	const auto half_width = eval_float("half_width");
	const auto radius = eval_float("radius");
	const auto center = eval_vec3("center");
	const auto type = eval_enum("solid_type");

	openvdb::FloatGrid::Ptr grid;

	switch (type) {
		default:
		case PLATONIC_SPHERE:
			grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(&center[0]), voxel_size, half_width);
			break;
		case PLATONIC_CUBE:
			grid = openvdb::tools::createLevelSetCube<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(&center[0]), voxel_size, half_width);
			break;
		case PLATONIC_TETRA:
			grid = openvdb::tools::createLevelSetTetrahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(&center[0]), voxel_size, half_width);
			break;
		case PLATONIC_OCTA:
			grid = openvdb::tools::createLevelSetOctahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(&center[0]), voxel_size, half_width);
			break;
		case PLATONIC_DODE:
			grid = openvdb::tools::createLevelSetDodecahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(&center[0]), voxel_size, half_width);
			break;
		case PLATONIC_ICOSA:
			grid = openvdb::tools::createLevelSetIcosahedron<openvdb::FloatGrid>(
			           radius, openvdb::Vec3f(&center[0]), voxel_size, half_width);
			break;
	}

	m_collection->free_all();
	build_vdb_prim(m_collection, grid);
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodePlatonic>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
