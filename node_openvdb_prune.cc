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
#include <openvdb/tools/Prune.h>

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NOM_OPERATEUR = "OpenVDB Prune";
static constexpr auto AIDE_OPERATEUR = "";

enum {
	VALUE = 0,
	INACTIVE = 1,
	LEVEL_SET = 2,
};

class NodePrune : public OperateurOpenVDB {

public:
	NodePrune(Noeud *noeud, const Context &contexte);

	const char *nom_entree(size_t /*index*/) override { return "VDB"; }
	const char *nom_sortie(size_t /*index*/) override { return "VDB"; }

	void execute(const Context &contexte, double temps) override;
};

class PruneOp {
	const int mode;
	const float tolerance;

public:
	PruneOp(int m, float t)
	    : mode(m)
	    , tolerance(t)
	{}

	template<typename GridType>
    void operator()(GridType &grid) const
    {
        using ValueT = typename GridType::ValueType;

        switch (mode) {
			case VALUE:
				openvdb::tools::prune(grid.tree(), ValueT(openvdb::zeroVal<ValueT>() + tolerance));
				break;
			case INACTIVE:
				openvdb::tools::pruneInactive(grid.tree());
				break;
			case LEVEL_SET:
				openvdb::tools::pruneLevelSet(grid.tree());
				break;
		}
    }
};

NodePrune::NodePrune(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(1);
	sorties(1);

	EnumProperty mode_enum;
	mode_enum.insert("Value",     VALUE);
	mode_enum.insert("Inactive",  INACTIVE);
	mode_enum.insert("Level Set", LEVEL_SET);

	add_prop("mode", "Mode", property_type::prop_enum);
	set_prop_enum_values(mode_enum);

	add_prop("tolerance", "Tolerance", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
}

void NodePrune::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	const auto mode = eval_int("mode");
	const auto tolerance = eval_float("tolerance");

	PruneOp op(mode, tolerance);

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);
		process_typed_grid(vdb_prim->getGrid(), vdb_prim->storage(), op);
	}
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodePrune>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
