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

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/util/NullInterrupter.h>

/* ************************************************************************** */

namespace {

struct SegmentActiveVoxels {
private:
    PrimitiveCollection * const m_collection;
    const bool m_append_number;

public:
    SegmentActiveVoxels(PrimitiveCollection &geo, bool appendNumber, openvdb::util::NullInterrupter&)
        : m_collection(&geo)
        , m_append_number(appendNumber)
    {}

    template<typename GridType>
    void operator()(const GridType& grid)
    {
         typedef typename GridType::Ptr   GridPtrType;

        std::vector<GridPtrType> segments;

        openvdb::tools::segmentActiveVoxels(grid, segments);

        for (size_t n = 0, N = segments.size(); n < N; ++n) {
            std::string name = grid.getName();

            if (m_append_number) {
                std::stringstream ss;
                ss << name << "_" << n;
                name = ss.str();
            }

			segments[n]->setName(name);
            build_vdb_prim(m_collection, segments[n]);
        }
    }
};

struct SegmentSDF {
private:
    PrimitiveCollection * const m_collection;
    const bool m_append_number;

public:
    SegmentSDF(PrimitiveCollection &collection, bool append_number, openvdb::util::NullInterrupter&)
        : m_collection(&collection)
        , m_append_number(append_number)
    {}

    template<typename GridType>
    void operator()(const GridType& grid)
    {
        typedef typename GridType::Ptr GridPtrType;

        std::vector<GridPtrType> segments;

        openvdb::tools::segmentSDF(grid, segments);

        for (size_t n = 0, N = segments.size(); n < N; ++n) {
            std::string name = grid.getName();

            if (m_append_number) {
                std::stringstream ss;
                ss << name << "_" << n;
                name = ss.str();
            }

			segments[n]->setName(name);
            build_vdb_prim(m_collection, segments[n]);
        }
    }
};

}  /* namespace */

/* ************************************************************************** */

static constexpr auto NOM_OPERATEUR = "OpenVDB Segment";
static constexpr auto AIDE_OPERATEUR = "";

class NodeOpenVDBSegment : public OperateurOpenVDB {
public:
	NodeOpenVDBSegment(Noeud *noeud, const Context &contexte)
	    : OperateurOpenVDB(noeud, contexte)
	{
		entrees(1);
		sorties(1);

		add_prop("append_number", "Append Segment Number to Grid Name", property_type::prop_bool);
		set_prop_default_value_bool(true);
	}

	const char *nom_entree(size_t /*index*/) override { return "input"; }
	const char *nom_sortie(size_t /*index*/) override { return "output"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override
	{
		entree(0)->requiers_collection(m_collection, contexte, temps);

		auto num_grids = 0;

		for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
			if (prim) {
				++num_grids;
			}
		}

		if (num_grids == 0) {
			this->ajoute_avertissement("No VDB grids to process.");
			return;
		}

		auto append_number = eval_bool("append_number");
		openvdb::util::NullInterrupter boss;

		PrimitiveCollection collection(m_collection->factory());

		SegmentActiveVoxels segmentActiveVoxels(collection, append_number, boss);
		SegmentSDF segmentSDF(collection, append_number, boss);
		std::vector<Primitive *> to_destroy;

		for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
			auto vdb = static_cast<VDBVolume *>(prim);

			const auto grid_class = vdb->getGrid().getGridClass();

			if (grid_class == openvdb::GRID_LEVEL_SET) {
				process_grid_real(vdb->getGrid(), vdb->storage(), segmentSDF);
			}
			else {
				process_typed_grid(vdb->getGrid(), vdb->storage(), segmentActiveVoxels);
			}

			to_destroy.push_back(prim);
		}

		m_collection->destroy(to_destroy);
		m_collection->merge_collection(collection);
	}
};

/* ************************************************************************** */

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeOpenVDBSegment>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
