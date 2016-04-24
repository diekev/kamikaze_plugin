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
#include <kamikaze/paramfactory.h>

#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Write";

class NodeWrite : public Node {
	std::string m_filename;
	int m_compression;
	bool m_save_as_half;

public:
	NodeWrite();
	~NodeWrite() = default;

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeWrite::NodeWrite()
    : Node(NODE_NAME)
{
	addInput("Primitive");
}

void NodeWrite::setUIParams(ParamCallback *cb)
{
	file_param(cb, "File Path", &m_filename);

	const char *compression_items[] = {
	    "zip", "blosc", "none", nullptr
	};

	enum_param(cb, "Compression", &m_compression, compression_items, 0);

	bool_param(cb, "Save as Half floats", &m_save_as_half, false);
}

void NodeWrite::process()
{
	auto prim = getInputPrimitive("Primitive");

	if (!prim || m_filename.empty()) {
		return;
	}

	auto ls = static_cast<VDBVolume *>(prim);

	openvdb::io::File file(m_filename);

	switch (m_compression) {
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

	if (m_save_as_half) {
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
