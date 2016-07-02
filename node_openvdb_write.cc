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

#include <kamikaze/nodes.h>

#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Write";

class NodeWrite : public Node {

public:
	NodeWrite();
	~NodeWrite() = default;

	void process() override;
};

NodeWrite::NodeWrite()
    : Node(NODE_NAME)
{
	addInput("VDB");

	EnumProperty compression_enum;
	compression_enum.insert("ZIP", 0);
	compression_enum.insert("Blosc", 1);
	compression_enum.insert("None", 2);

	add_prop("Compression", property_type::prop_enum);
	set_prop_enum_values(compression_enum);

	add_prop("Save as Half floats", property_type::prop_bool);
}

void NodeWrite::process()
{
	auto prim = getInputPrimitive("VDB");

	const auto filename = eval_string("File Path");

	if (!prim || filename.empty()) {
		return;
	}

	const auto compression = eval_enum("Compression");
	const auto save_as_half = eval_bool("Save as Half floats");

	auto ls = static_cast<VDBVolume *>(prim);

	openvdb::io::File file(filename);

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

	if (save_as_half) {
		ls->getGridPtr()->setSaveFloatAsHalf(true);
	}

	openvdb::MetaMap metadatas;
    metadatas.insertMeta("creator", openvdb::StringMetadata("Kamikaze/OpenVDBWriter"));

	openvdb::GridCPtrVec grids;

	grids.push_back(ls->getGridPtr()->deepCopyGrid());

	file.write(grids, metadatas);
}

static Node *new_write_node()
{
	return new NodeWrite;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_write_node);
}

}
