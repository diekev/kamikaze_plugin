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

static constexpr auto NOM_OPERATEUR = "OpenVDB From Polygons";
static constexpr auto AIDE_OPERATEUR = "";

class NodeFromPolygons : public OperateurOpenVDB {
public:
	NodeFromPolygons(Noeud *noeud, const Context &contexte);
	~NodeFromPolygons() = default;

	const char *nom_entree(size_t index) override
	{
		if (index == 0) {
			return "input";
		}

		return "reference VDB";
	}

	const char *nom_sortie(size_t /*index*/) override { return "output"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;
	bool update_properties() override;
};

enum {
	VOXEL_WORLD = 0,
	VOXEL_AXIS_X = 1,
	VOXEL_AXIS_Y = 2,
	VOXEL_AXIS_Z = 3,
	VOXEL_AXIS_LONGEST = 4,
};

NodeFromPolygons::NodeFromPolygons(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(2);
	sorties(1);

	add_prop("distanceField", "Output Distance Field", property_type::prop_bool);
	set_prop_default_value_bool(true);
	set_prop_tooltip("Enable / disable the level set output.");

	add_prop("distanceFieldGridName", "Distance VDB Name", property_type::prop_string);
	set_prop_default_value_string("surface");
	set_prop_tooltip("Outputs a distance field / level set grid. Voxels "
	                 "in the narrow band are made active. (A grid name can optionally be "
		             "specified in the string field)");

	add_prop("fogVolume", "Output Fog Volume", property_type::prop_bool);
	set_prop_tooltip("Enable / disable the fog volume output.");

	add_prop("fogVolumeGridName", "Fog VDB Name", property_type::prop_string);
	set_prop_default_value_string("density");
	set_prop_tooltip("Outputs the fog volume grid. Generated from the signed "
	                 "distance field / level set, the interior narrow band is "
		             "transformed into a 0 to 1 gradient and the remaining interior "
		             "values are set to 1. Exterior values and the background are "
		             "set to 0. The interior is still a sparse representations but "
		             "the values are active.");

	/* ------------------------- Conversion settings ------------------------ */

	EnumProperty items;
	items.insert("Size In World Units", VOXEL_WORLD);
	items.insert("Count Along X Axis", VOXEL_AXIS_X);
	items.insert("Count Along Y Axis", VOXEL_AXIS_Y);
	items.insert("Count Along Z Axis", VOXEL_AXIS_Z);
	items.insert("Count Along Longest Axis", VOXEL_AXIS_LONGEST);

	add_prop("sizeOrCount", "Voxel", property_type::prop_enum);
	set_prop_enum_values(items);
	set_prop_tooltip("Specify the voxel size in world units or voxel count along an axis");

	add_prop("voxel_size", "Voxel Size", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(0.1f);
	set_prop_tooltip("Uniform voxel size in world units of the generated level set.");

	add_prop("voxelCount", "Voxel Count", property_type::prop_int);
	set_prop_min_max(1, 500);
	set_prop_default_value_int(100);
	set_prop_tooltip("Specify the voxel count along an axis.\n"
	                 "Note that the resulting voxel count might be off by one voxel"
		             " due to roundoff errors during the conversion process.");

	add_prop("worldSpaceUnits", "Use World Space Units for Narrow Band", property_type::prop_bool);

	add_prop("interior_band", "Interior Band", property_type::prop_int);
	set_prop_min_max(1, 10);
	set_prop_default_value_int(3);
	set_prop_tooltip("Specify the width of the interior (d < 0) portion of the narrow band. "
	                 "(3 voxel units is optimal for level set operations.)");

	add_prop("exterior_band", "Exterior Band", property_type::prop_int);
	set_prop_min_max(1, 10);
	set_prop_default_value_int(3);
	set_prop_tooltip("Specify the width of the exterior (d >= 0) portion of the narrow band. "
	                 "(3 voxel units is optimal for level set operations.)");

	add_prop("interior_band_ws", "Interior Band", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Specify the width of the interior (d < 0) portion of the narrow band.");

	add_prop("exterior_band_ws", "Exterior Band", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Specify the width of the exterior (d >= 0) portion of the narrow band.");

	add_prop("fillInterior", "Fill Interior", property_type::prop_bool);
	set_prop_tooltip("Extract signed distances for all interior voxels, this "
	                 "operation is going to densify the interior of the model. "
		             "Requires a closed watertight mesh.");

	add_prop("unsignedDist", "Unsigned Distance Field", property_type::prop_bool);
	set_prop_tooltip("Generate an unsigned distance field. This operation "
	                 "will work on any mesh, i.e. does not require a closed "
		             "watertight mesh.");

	/* TODO: attributes */
}

bool NodeFromPolygons::update_properties()
{
	const auto refexists = entree(1)->est_connectee();

	// Conversion
    const bool wsUnits = eval_bool("worldSpaceUnits");
    const bool unsignedDist = eval_bool("unsignedDist");

    // Voxel size or voxel count menu
	const bool countMenu = (eval_enum("sizeOrCount") != VOXEL_WORLD);
	set_prop_visible("voxel_size", !countMenu && !refexists);
    set_prop_visible("voxelCount", countMenu && !refexists);

	set_prop_visible("interior_band", !wsUnits);
	set_prop_visible("exterior_band", !wsUnits);
	set_prop_visible("interior_band_ws", wsUnits);
	set_prop_visible("exterior_band_ws", wsUnits);

    set_prop_visible("fillInterior", !unsignedDist);

    // Output
    set_prop_visible("distanceFieldGridName", eval_bool("distanceField"));
    set_prop_visible("fogVolumeGridName", eval_bool("fogVolume") && !unsignedDist);
    set_prop_visible("fogVolume", !unsignedDist);

	return true;
}

void NodeFromPolygons::execute(const Context &contexte, double temps)
{
	const auto voxel_size = eval_float("voxel_size");
	const auto int_band = eval_int("interior_band");
	const auto ext_band = eval_int("exterior_band");

	entree(0)->requiers_collection(m_collection, contexte, temps);

	std::vector<Primitive *> converted_prims;

	auto transform = openvdb::math::Transform::createLinearTransform(voxel_size);

	PrimitiveCollection tmp_collection(m_collection->factory());
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

		build_vdb_prim(&tmp_collection, grid);
	}

	/* Remove the converted meshes from the collection. */
	m_collection->destroy(converted_prims);
	m_collection->merge_collection(tmp_collection);
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeFromPolygons>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
