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

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NOM_OPERATEUR = "OpenVDB Create";
static constexpr auto AIDE_OPERATEUR = "";

class NodeCreate : public OperateurOpenVDB {
public:
	NodeCreate(Noeud *noeud, const Context &contexte);

	const char *nom_sortie(size_t /*index*/) override { return "VDB"; }

	void execute(const Context &contexte, double temps) override;
};

NodeCreate::NodeCreate(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	sorties(1);

	add_prop("grid_name", "Grid Name", property_type::prop_string);
	set_prop_default_value_string("VDB Grid");

	EnumProperty storage_enum;
	storage_enum.insert("Float", 0);
	storage_enum.insert("Double", 1);
	storage_enum.insert("Int32", 2);
	storage_enum.insert("Int64", 3);
	storage_enum.insert("Vec3I", 4);
	storage_enum.insert("Vec3s", 5);
	storage_enum.insert("Vec3d", 6);

	add_prop("storage", "Storage", property_type::prop_enum);
	set_prop_enum_values(storage_enum);

	EnumProperty vectype_enum;
	vectype_enum.insert("Invariant", 0);
	vectype_enum.insert("Covariant", 1);
	vectype_enum.insert("Covariant Normalize", 2);
	vectype_enum.insert("Contravariant Relative", 3);
	vectype_enum.insert("Contravariant Absolute", 4);

	add_prop("vector_type", "Vector Type", property_type::prop_enum);
	set_prop_enum_values(vectype_enum);
	set_prop_tooltip("The type of a vector determines how transforms are applied to it\n"
	                 "Invariant:\n"
	                 "    Does not transform (e.g., tuple, uvw, color)\n"
	                 "Covariant:\n"
	                 "    Apply inverse-transpose transformation: ignores translation, (e.g., gradient/normal)\n"
	                 "Covariant Normalize:\n"
	                 "    Apply inverse-transpose transformation: ignores translation, vectors are renormalized (e.g., unit normal)\n"
	                 "Contravariant Relative:\n"
	                 "    Apply \"regular\" transformation: ignores translation (e.g., displacement, velocity, acceleration)\n"
	                 "Contravariant Absolute:\n"
	                 "    Apply \"regular\" transformation: vector translates (e.g., position)");


	EnumProperty gridclass_enum;
	gridclass_enum.insert("None", 0);
	gridclass_enum.insert("Level Set", 1);
	gridclass_enum.insert("Fog Volume", 2);
	gridclass_enum.insert("Staggered", 3);

	add_prop("grid_class", "Grid Class", property_type::prop_enum);
	set_prop_enum_values(gridclass_enum);

	add_prop("background", "Background Value", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(0.0f);
	set_prop_tooltip("Unique value returned when accessing a location in space"
	                 " that does not resolve to a voxel or a tile.");

	add_prop("voxel_size", "Voxel Size", property_type::prop_float);
	set_prop_min_max(0.01f, 10.0f);
	set_prop_default_value_float(0.1f);
	set_prop_tooltip("Uniform voxel size in world units.");
}

void NodeCreate::execute(const Context &contexte, double temps)
{
	const auto background = eval_float("background");
	const auto voxel_size = eval_float("voxel_size");
	const auto gridclass = eval_enum("grid_class");
	const auto storage = eval_enum("storage");
	const auto vectype = eval_enum("vector_type");
	const auto gridname = eval_string("grid_name");

	int storage_type = storage;

	/* Force a specific type for some grid classes to avoid issues */
	if (gridclass == openvdb::GridClass::GRID_LEVEL_SET) {
		if (!is_elem(storage, GRID_STORAGE_FLOAT, GRID_STORAGE_DOUBLE)) {
			storage_type = GRID_STORAGE_FLOAT;
		}
	}
	else if (gridclass == openvdb::GridClass::GRID_FOG_VOLUME) {
		if (!is_elem(storage, GRID_STORAGE_FLOAT, GRID_STORAGE_DOUBLE)) {
			storage_type = GRID_STORAGE_FLOAT;
		}
	}
	else if (gridclass == openvdb::GridClass::GRID_STAGGERED) {
		if (!is_elem(storage, GRID_STORAGE_VEC3D, GRID_STORAGE_VEC3S, GRID_STORAGE_VEC3I)) {
			storage_type = GRID_STORAGE_VEC3S;
		}
	}

	openvdb::GridBase::Ptr grid;

	switch (storage_type) {
		case GRID_STORAGE_FLOAT:
			grid = openvdb::FloatGrid::create(background);
			break;
		case GRID_STORAGE_DOUBLE:
			grid = openvdb::DoubleGrid::create(background);
			break;
		case GRID_STORAGE_BOOL:
			grid = openvdb::BoolGrid::create(background);
			break;
		case GRID_STORAGE_INT32:
			grid = openvdb::Int32Grid::create(background);
			break;
		case GRID_STORAGE_INT64:
			grid = openvdb::Int64Grid::create(background);
			break;
		case GRID_STORAGE_VEC3I:
			grid = openvdb::Vec3IGrid::create(openvdb::Vec3I(background));
			break;
		case GRID_STORAGE_VEC3S:
			grid = openvdb::Vec3SGrid::create(openvdb::Vec3s(background));
			break;
		case GRID_STORAGE_VEC3D:
			grid = openvdb::Vec3DGrid::create(openvdb::Vec3d(background));
			break;
	}

	auto transform = openvdb::math::Transform::createLinearTransform(voxel_size);

	grid->setName(gridname);
	grid->setTransform(transform);
	grid->setVectorType(static_cast<openvdb::VecType>(vectype));
	grid->setGridClass(static_cast<openvdb::GridClass>(gridclass));

	m_collection->free_all();
	build_vdb_prim(m_collection, grid);
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeCreate>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
