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

#include "util_openvdb_process.h"
#include "volumebase.h"

static auto begin(const openvdb::io::File &file)
{
	return file.beginName();
}

static auto end(const openvdb::io::File &file)
{
	return file.endName();
}

static constexpr auto NODE_NAME = "OpenVDB Read";

class NodeOpenVDBRead : public Node {

public:
	NodeOpenVDBRead();

	void process() override;
};

NodeOpenVDBRead::NodeOpenVDBRead()
    : Node(NODE_NAME)
{
	addOutput("VDB");

	add_prop("File Path", property_type::prop_input_file);

	add_prop("Grid Name", property_type::prop_string);
	set_prop_default_value_string("density");
	set_prop_tooltip("Name of the grid to lookup");
}

void NodeOpenVDBRead::process()
{
	const auto filename = eval_string("File Path");

	if (filename.empty()) {
		setOutputCollection("VDB", nullptr);
		return;
	}

	openvdb::initialize();

	openvdb::io::File file(filename);

	if (!file.open()) {
		setOutputCollection("VDB", nullptr);
		std::cerr << "Unable to open file \'" << filename << "\'\n";
		return;
	}

	std::cerr << "Grids in file:\n";
	for (auto iter = begin(file), eiter = end(file); iter != eiter; ++iter) {
		std::cerr << *iter << "\n";
	}

	const auto gridname = eval_string("Grid Name");

	if (gridname.size() == 0 || !file.hasGrid(gridname)) {
		setOutputCollection("VDB", nullptr);
		std::cerr << "Cannot lookup \'" << gridname << "\' in file.\n";
		return;
	}

	auto grid = file.readGrid(gridname);

	file.close();

	build_vdb_prim(m_collection, grid);
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBRead);
}

}
