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

#include <openvdb/tools/LevelSetFilter.h>

#include "volumebase.h"

static constexpr auto NOM_OPERATEUR = "OpenVDB Filter Level Set";
static constexpr auto AIDE_OPERATEUR = "";

class NodeFilterLevelSet : public OperateurOpenVDB {
public:
	NodeFilterLevelSet(Noeud *noeud, const Context &contexte);
	~NodeFilterLevelSet() = default;

	const char *nom_entree(size_t index) override
	{
		switch (index) {
			default:
			case 0:
				return "VDB";
			case 1:
				return "VDB Mask";
		}
	}

	const char *nom_sortie(size_t /*index*/) override { return "VDB"; }

	void execute(const Context &contexte, double temps) override;
};

enum {
	LS_FILTER_ACC_FISRT = 0,
	LS_FILTER_ACC_SECOND,
	LS_FILTER_ACC_THIRD,
	LS_FILTER_ACC_WENO5,
	LS_FILTER_ACC_HJWENO5,
};

enum {
	LS_FILTER_MEDIAN = 0,
	LS_FILTER_MEAN,
	LS_FILTER_GAUSSIAN,
	LS_FILTER_MEAN_CURV,
	LS_FILTER_LAPLACIAN,
	LS_FILTER_OFFSET,
};

NodeFilterLevelSet::NodeFilterLevelSet(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(2);
	sorties(1);

	EnumProperty type_enum;
	type_enum.insert("Median",         LS_FILTER_MEDIAN);
	type_enum.insert("Mean",           LS_FILTER_MEAN);
	type_enum.insert("Gaussian",       LS_FILTER_GAUSSIAN);
	type_enum.insert("Mean Curvature", LS_FILTER_MEAN_CURV);
	type_enum.insert("Laplacian",      LS_FILTER_LAPLACIAN);
	type_enum.insert("Offset",         LS_FILTER_OFFSET);

	add_prop("filter_type", "Filter Type", property_type::prop_enum);
	set_prop_enum_values(type_enum);

	EnumProperty accuracy_enum;
	accuracy_enum.insert("First Bias",    LS_FILTER_ACC_FISRT);
	accuracy_enum.insert("Second Bias",   LS_FILTER_ACC_SECOND);
	accuracy_enum.insert("Third Bias",    LS_FILTER_ACC_THIRD);
	accuracy_enum.insert("WENO5 Bias",    LS_FILTER_ACC_WENO5);
	accuracy_enum.insert("HJ WENO5 Bias", LS_FILTER_ACC_HJWENO5);

	add_prop("accuracy", "Accuracy", property_type::prop_enum);
	set_prop_enum_values(accuracy_enum);

	add_prop("iterations", "Iterations", property_type::prop_int);
	set_prop_min_max(1, 10);
	set_prop_default_value_int(1);

	add_prop("width", "Width", property_type::prop_int);
	set_prop_min_max(1, 10);
	set_prop_default_value_int(1);

	add_prop("offset", "Offset", property_type::prop_float);
	set_prop_min_max(1, 10);
	set_prop_default_value_float(1.0f);
}

void NodeFilterLevelSet::execute(const Context &contexte, double temps)
{
	using namespace openvdb;

	entree(0)->requiers_collection(m_collection, contexte, temps);

	const auto type = eval_enum("filter_type");
	const auto accuracy = eval_enum("accuracy");
	const auto iterations = eval_int("iterations");
	const auto width = eval_int("width");
	const auto offset = eval_float("offset");

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);

		if (!is_level_set(vdb_prim)) {
			continue;
		}

		auto ls_grid = gridPtrCast<FloatGrid>(vdb_prim->getGridPtr());

		FloatGrid *mask = nullptr;
		auto mask_prim = entree(1)->requiers_collection(nullptr, contexte, temps);

		if (mask_prim) {
			auto mask_ls = static_cast<VDBVolume *>(mask_prim->primitives()[0]);
			mask = (gridPtrCast<FloatGrid>(mask_ls->getGridPtr())).get();
		}

		typedef tools::LevelSetFilter<FloatGrid> Filter;

		Filter filter(*ls_grid);

		filter.setTemporalScheme(math::TVD_RK1);

		switch (accuracy) {
			case LS_FILTER_ACC_FISRT:   filter.setSpatialScheme(math::FIRST_BIAS);   break;
			case LS_FILTER_ACC_SECOND:  filter.setSpatialScheme(math::SECOND_BIAS);  break;
			case LS_FILTER_ACC_THIRD:   filter.setSpatialScheme(math::THIRD_BIAS);   break;
			case LS_FILTER_ACC_WENO5:   filter.setSpatialScheme(math::WENO5_BIAS);   break;
			case LS_FILTER_ACC_HJWENO5: filter.setSpatialScheme(math::HJWENO5_BIAS); break;
		}

		switch (type) {
			case LS_FILTER_MEDIAN:
				for (int i = 0; i < iterations; ++i) {
					filter.median(width, mask);
				}
				break;
			case LS_FILTER_MEAN:
				for (int i = 0; i < iterations; ++i) {
					filter.mean(width, mask);
				}
				break;
			case LS_FILTER_GAUSSIAN:
				for (int i = 0; i < iterations; ++i) {
					filter.gaussian(width, mask);
				}
				break;
			case LS_FILTER_MEAN_CURV:
				for (int i = 0; i < iterations; ++i) {
					filter.meanCurvature(mask);
				}
				break;
			case LS_FILTER_LAPLACIAN:
				for (int i = 0; i < iterations; ++i) {
					filter.laplacian(mask);
				}
				break;
			case LS_FILTER_OFFSET:
				for (int i = 0; i < iterations; ++i) {
					filter.offset(offset, mask);
				}
				break;
		}

		vdb_prim->setGrid(ls_grid);
	}
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeFilterLevelSet>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
