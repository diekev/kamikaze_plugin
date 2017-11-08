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
#include "node_openvdb.h"

#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>

#include "volumebase.h"

static constexpr auto NOM_OPERATEUR = "OpenVDB To Polygons";
static constexpr auto AIDE_OPERATEUR = "";

class NodeToPolygons : public OperateurOpenVDB {

public:
	NodeToPolygons(Noeud *noeud, const Context &contexte);
	~NodeToPolygons() = default;

	const char *nom_entree(size_t /*index*/) override { return "VDB"; }
	const char *nom_sortie(size_t /*index*/) override { return "Mesh"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;
};

NodeToPolygons::NodeToPolygons(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(1);
	sorties(1);

	add_prop("iso_value", "Iso Value", property_type::prop_float);
	set_prop_min_max(-1.0f, 1.0f);
	set_prop_tooltip("Crossing point value considered as the surface.");

	add_prop("adaptivity", "Adaptivity", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_tooltip("Determine how closely the isosurface is matched by the resulting mesh.");
}

void NodeToPolygons::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	const auto iso_value = eval_float("iso_value");
	const auto adaptivity = eval_float("adaptivity");

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);

//		if (!is_level_set(vdb_prim)) {
//			continue;
//		}

		auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(vdb_prim->getGridPtr());

		openvdb::tools::VolumeToMesh mesher(iso_value, adaptivity);
		mesher(*grid);

		auto mesh = static_cast<Mesh *>(m_collection->build("Mesh"));

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
	}
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeToPolygons>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
