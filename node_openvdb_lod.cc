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

#include <openvdb/tools/MultiResGrid.h>
#include <openvdb/util/NullInterrupter.h>

#include "util_string.h"

/* ************************************************************************** */

namespace {

template<openvdb::Index Order>
struct MultiResGridFractionalOp {
    MultiResGridFractionalOp(float f)
	    : level(f)
	{}

    template<typename GridType>
    void operator()(const GridType& grid)
    {
        if ( level <= 0.0f ) {
            outputGrid = typename GridType::Ptr( new GridType(grid) );
        }
		else {
            const size_t levels = openvdb::math::Ceil(level) + 1;
            typedef typename GridType::TreeType TreeT;
            openvdb::tools::MultiResGrid<TreeT> mrg( levels, grid );
            outputGrid = mrg.template createGrid<Order>( level );
        }
    }

    const float level;
    openvdb::GridBase::Ptr outputGrid;
};

template<openvdb::Index Order>
struct MultiResGridRangeOp {
    MultiResGridRangeOp(float start_, float end_, float step_, openvdb::util::NullInterrupter& boss_)
        : start(start_)
	    , end(end_)
	    , step(step_)
	    , outputGrids()
	    , boss(&boss_)
    {}

    template<typename GridType>
    void operator()(const GridType& grid)
    {
        if (end > 0.0f) {
            const size_t levels = openvdb::math::Ceil(end) + 1;
            typedef typename GridType::TreeType TreeT;
            openvdb::tools::MultiResGrid<TreeT> mrg(levels, grid);

            // inclusive range
            for (float level = start; !(level > end); level += step) {
				if (boss->wasInterrupted()) {
					break;
				}

                outputGrids.push_back(mrg.template createGrid<Order>(level));
            }
        }
    }

    const float start, end, step;
    std::vector<openvdb::GridBase::Ptr> outputGrids;
    openvdb::util::NullInterrupter * const boss;
};

struct MultiResGridIntegerOp {
    MultiResGridIntegerOp(size_t n)
	    : levels(n)
	{}

    template<typename GridType>
    void operator()(const GridType& grid)
    {
        typedef typename GridType::TreeType TreeT;
        openvdb::tools::MultiResGrid<TreeT> mrg( levels, grid );
        outputGrids = mrg.grids();
    }

    const size_t levels;
    openvdb::GridPtrVecPtr outputGrids;
};

inline bool is_valid_range(float start, float end, float step)
{
    if (start < 0.0f || !(step > 0.0f) || end < 0.0f) {
        return false;
    }

    return !(start > end);
}

}  /* unnamed namespace */

/* ************************************************************************** */

enum {
	LOD_SINGLE  = 0,
	LOD_RANGE   = 1,
	LOD_PYRAMID = 2,
};

static constexpr auto NOM_OPERATEUR = "OpenVDB LOD";
static constexpr auto AIDE_OPERATEUR = "";

class NodeOpenVDBLOD : public OperateurOpenVDB {
public:
	NodeOpenVDBLOD(Noeud *noeud, const Context &contexte)
	    : OperateurOpenVDB(noeud, contexte)
	{
		entrees(1);
		sorties(1);

		EnumProperty lod_enum;
		lod_enum.insert("Single Level", LOD_SINGLE);
		lod_enum.insert("Level Range", LOD_RANGE);
		lod_enum.insert("LOD Pyramid", LOD_PYRAMID);

		add_prop("mode", "LOD Mode", property_type::prop_enum);
		set_prop_enum_values(lod_enum);

		add_prop("level", "Level", property_type::prop_float);
		set_prop_min_max(0.0f, 10.0f);
		set_prop_default_value_float(1.0f);
		set_prop_tooltip("Specify which level to produce.\n"
		                 "Level 0 is the highest-resolution level.");

		add_prop("range", "Range", property_type::prop_vec3);
		set_prop_min_max(0.0f, 10.0f);
		set_prop_default_value_vec3(glm::vec3{0.0f, 2.0f, 1.0f});
		set_prop_tooltip("Specify the inclusive range [start, end, step]");

		add_prop("count", "Count", property_type::prop_int);
		set_prop_min_max(2.0f, 10.0f);
		set_prop_default_value_int(2);
		set_prop_tooltip("Number of levels\n"
		                 "Each level is half the resolution of the previous level.");

		add_prop("preserve_names", "Preserve Grid Names", property_type::prop_bool);
		set_prop_default_value_bool(false);
		set_prop_tooltip("Reuse the name of the input VDB grid.\n"
		                 "Only available when a single Level is generated.");
	}

	const char *nom_entree(size_t /*index*/) override { return "input"; }
	const char *nom_sortie(size_t /*index*/) override { return "output"; }

	bool update_properties() override
	{
		const auto lod_mode = eval_int("mode");

	    set_prop_visible("level", lod_mode == LOD_SINGLE);
	    set_prop_visible("preserve_names", lod_mode == LOD_SINGLE);
	    set_prop_visible("range", lod_mode == LOD_RANGE);
	    set_prop_visible("count", lod_mode == LOD_PYRAMID);

		return true;
	}

	void execute(const Context &contexte, double temps) override
	{
		std::vector<std::string> skipped;
		std::vector<Primitive *> to_destroy;
		openvdb::util::NullInterrupter boss;

		entree(0)->requiers_collection(m_collection, contexte, temps);

		const auto lod_mode = eval_int("mode");

		if (lod_mode == LOD_SINGLE) {
			const auto reuse_name = eval_bool("preserve_names") > 0;
			MultiResGridFractionalOp<1> op(eval_float("level"));

			for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
				auto vdb = static_cast<VDBVolume *>(prim);

				if (!vdb->getGrid().transform().isLinear()) {
					skipped.push_back(vdb->getGrid().getName());
					continue;
				}

				/* TODO: process_typed_grid should not process bool types */
				process_grid_real(vdb->getGrid(), vdb->storage(), op);

				if (reuse_name) {
					op.outputGrid->setName(vdb->getGrid().getName());
				}

				build_vdb_prim(m_collection, op.outputGrid);

				to_destroy.push_back(prim);
			}
		}
		else if (lod_mode == LOD_RANGE) {
			const auto range = eval_vec3("range");
			const auto start = range[0];
			const auto end = range[1];
			const auto step = range[2];

			if (!is_valid_range(start, end, step)) {
				this->ajoute_avertissement("Invalid range, make sure that start <= end"
				                  " and the step size is a positive number.");
				return;
			}

			MultiResGridRangeOp<1> op(start, end, step, boss);

			for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
				auto vdb = static_cast<VDBVolume *>(prim);

				if (!vdb->getGrid().transform().isLinear()) {
					skipped.push_back(vdb->getGrid().getName());
					continue;
				}

				/* TODO: process_typed_grid should not process bool types */
				process_grid_real(vdb->getGrid(), vdb->storage(), op);

				for (size_t i = 0; i < op.outputGrids.size(); ++i) {
					build_vdb_prim(m_collection, op.outputGrids[i]);
				}

				to_destroy.push_back(prim);
			}
		}
		else if (lod_mode == LOD_PYRAMID) {
			MultiResGridIntegerOp op(eval_int("count"));

			for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
				auto vdb = static_cast<VDBVolume *>(prim);

				if (!vdb->getGrid().transform().isLinear()) {
					skipped.push_back(vdb->getGrid().getName());
					continue;
				}

				/* TODO: process_typed_grid should not process bool types */
				process_grid_real(vdb->getGrid(), vdb->storage(), op);

				for (size_t i = 0; i < op.outputGrids->size(); ++i) {
					build_vdb_prim(m_collection, op.outputGrids->at(i));
				}

				to_destroy.push_back(prim);
			}
		}
		else {
			this->ajoute_avertissement("Invalid LOD option.");
		}

		m_collection->destroy(to_destroy);

		if (!skipped.empty()) {
			this->ajoute_avertissement("Unable to process grid(s): " + join(skipped, ", "));
		}
	}
};

/* ************************************************************************** */

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeOpenVDBLOD>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
