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

#include "volumebase.h"

static constexpr auto NOM_OPERATEUR = "OpenVDB Write";
static constexpr auto AIDE_OPERATEUR = "";

class NodeWrite : public OperateurOpenVDB {

public:
	NodeWrite(Noeud *noeud, const Context &contexte);
	~NodeWrite() = default;

	const char *nom_entree(size_t /*index*/) override { return "VDB"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;
};

NodeWrite::NodeWrite(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(1);

	EnumProperty compression_enum;
	compression_enum.insert("ZIP", 0);
	compression_enum.insert("Blosc", 1);
	compression_enum.insert("None", 2);

	add_prop("compression", "Compression", property_type::prop_enum);
	set_prop_enum_values(compression_enum);

	add_prop("half_floats", "Save as Half floats", property_type::prop_bool);
}

void NodeWrite::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	const auto filename = eval_string("File Path");

	if (filename.empty()) {
		return;
	}

	const auto compression = eval_enum("compression");
	const auto save_as_half = eval_bool("half_floats");

	openvdb::io::File file(filename);

	openvdb::GridCPtrVec grids;

	openvdb::MetaMap metadatas;
    metadatas.insertMeta("creator", openvdb::StringMetadata("Kamikaze/OpenVDBWriter"));

	switch (compression) {
		default:
		case 0:
			file.setCompression(openvdb::io::COMPRESS_ZIP);
			break;
		case 1:
			file.setCompression(openvdb::io::COMPRESS_BLOSC);
			break;
		case 2:
			file.setCompression(openvdb::io::COMPRESS_NONE);
			break;
	}

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);

		if (save_as_half) {
			vdb_prim->getGridPtr()->setSaveFloatAsHalf(true);
		}

		grids.push_back(vdb_prim->getGridPtr()->deepCopyGrid());
	}

	file.write(grids, metadatas);
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeWrite>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
