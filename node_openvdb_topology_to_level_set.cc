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

#include <openvdb/tools/TopologyToLevelSet.h>
#include <openvdb/tools/LevelSetUtil.h>

#include "util_openvdb_process.h"
#include "volumebase.h"

/* ************************************************************************** */

enum {
	NAME_KEEP    = 0,
	NAME_APPEND  = 1,
	NAME_REPLACE = 2,
};

namespace {

class Converter {
	PrimitiveCollection * const m_collection;
    openvdb::util::NullInterrupter * const m_boss;

public:
    float band_width_world = 0;
    int band_width_voxels = 3;
	int closing_width = 1;
	int dilation = 0;
	int smoothing_steps = 0;
	int output_name = NAME_KEEP;
    bool world_space_units = false;
    std::string custom_name = "vdb";

    Converter(PrimitiveCollection &collection, openvdb::util::NullInterrupter &boss)
        : m_collection(&collection)
        , m_boss(&boss)
    {}

    template<typename GridType>
    void operator()(const GridType &grid)
    {
        auto bandWidth = band_width_voxels;

        if (world_space_units) {
            bandWidth = int(openvdb::math::Round(band_width_world / grid.transform().voxelSize()[0]));
        }

        auto sdfGrid = openvdb::tools::topologyToLevelSet(
           grid, bandWidth, closing_width, dilation, smoothing_steps, m_boss);

        auto name = grid.getName();

		if (output_name == NAME_APPEND) {
			name += custom_name;
		}
		else if (output_name == NAME_REPLACE) {
			name = custom_name;
		}

		sdfGrid->setName(name);

        build_vdb_prim(m_collection, sdfGrid);
    }
};

} // unnamed namespace

/* ************************************************************************** */

static constexpr auto NOM_OPERATEUR = "OpenVDB Topology To Level Set";
static constexpr auto AIDE_OPERATEUR = "";

class NodeOpenVDBTopologyToLevelSet : public OperateurOpenVDB {
public:
	NodeOpenVDBTopologyToLevelSet(Noeud *noeud, const Context &contexte);
	~NodeOpenVDBTopologyToLevelSet() = default;

	const char *nom_entree(size_t /*index*/) override { return "input"; }
	const char *nom_sortie(size_t /*index*/) override { return "output"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;

	bool update_properties() override;
};

NodeOpenVDBTopologyToLevelSet::NodeOpenVDBTopologyToLevelSet(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(1);
	sorties(1);

	EnumProperty rename_items;
	rename_items.insert("Keep Incoming VDB Names", NAME_KEEP);
	rename_items.insert("Custom Append", NAME_APPEND);
	rename_items.insert("Custom Replace", NAME_REPLACE);

	add_prop("output_name", "Output Name", property_type::prop_enum);
	set_prop_enum_values(rename_items);
	set_prop_tooltip("Rename output grid(s)");

	add_prop("custom_name", "Custom Name", property_type::prop_string);
	set_prop_tooltip("Used to rename the input grids");

	/* Narrow-band width */
	add_prop("use_world_space", "Use World Space for Band", property_type::prop_bool);

	add_prop("half_band", "Half-Band in Voxels", property_type::prop_int);
	set_prop_default_value_int(3);
	set_prop_min_max(1, 10);
	set_prop_tooltip("Specify the half width of the narrow band. "
	                 "(3 voxel units is optimal for level set operations.)");

	add_prop("half_band_ws", "Half-Band in World", property_type::prop_float);
	set_prop_default_value_float(1.0f);
	set_prop_min_max(1e-5f, 10.0f);
	set_prop_tooltip("Specify the half width of the narrow band.");

	add_prop("dilation", "Voxel Dilation", property_type::prop_int);
	set_prop_default_value_int(0);
	set_prop_min_max(0, 10);
	set_prop_tooltip("Expands the filled voxel region by the specified "
	                 "number of voxels.");

	add_prop("closing", "Closing Width", property_type::prop_int);
	set_prop_default_value_int(1);
	set_prop_min_max(1, 10);
	set_prop_tooltip("First expand the filled voxel region, then shrink it "
	                 "by the specified number of voxels. This causes holes "
                     "and valleys to be filled.");

	add_prop("smoothing", "Smoothing Steps", property_type::prop_int);
	set_prop_default_value_int(0);
	set_prop_min_max(0, 10);
	set_prop_tooltip("Number of smoothing interations");
}

bool NodeOpenVDBTopologyToLevelSet::update_properties()
{
	const auto ws_units = eval_bool("use_world_space");
    set_prop_visible("half_band", !ws_units);
    set_prop_visible("half_band_ws", ws_units);

    const auto use_custom_name = (eval_enum("output_name") != NAME_KEEP);
    set_prop_visible("custom_name", use_custom_name);

	return true;
}

void NodeOpenVDBTopologyToLevelSet::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	PrimitiveCollection tmp_collection(m_collection->factory());
	openvdb::util::NullInterrupter boss;

	Converter converter(tmp_collection, boss);
    converter.world_space_units = eval_bool("use_world_space");
    converter.band_width_world = eval_float("half_band_ws");
    converter.band_width_voxels = eval_int("half_band");
    converter.closing_width = eval_int("closing");
    converter.dilation = eval_int("dilation");
    converter.smoothing_steps = eval_int("smoothing");
    converter.output_name = eval_enum("output_name");
    converter.custom_name = eval_string("custom_name");

	primitive_iterator iter(m_collection, VDBVolume::id);

    if (!iter.get()) {
		this->ajoute_avertissement("No VDB grids to process.");
        return;
    }

	std::vector<Primitive *> to_destroy;

	for (auto prim : iter) {
		auto vdb = static_cast<VDBVolume *>(prim);
		process_typed_grid_topology(vdb->getGrid(), vdb->storage(), converter);

		to_destroy.push_back(prim);
	}

	m_collection->destroy(to_destroy);
	m_collection->merge_collection(tmp_collection);
}

/* ************************************************************************** */

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeOpenVDBTopologyToLevelSet>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
