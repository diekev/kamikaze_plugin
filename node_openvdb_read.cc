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
	std::string m_filename = "";
	QString m_gridname = "";

public:
	NodeOpenVDBRead();

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeOpenVDBRead::NodeOpenVDBRead()
    : Node(NODE_NAME)
{
	addOutput("VDB");
}

void NodeOpenVDBRead::setUIParams(ParamCallback *cb)
{
	file_param(cb, "File Path", &m_filename);

	string_param(cb, "Grid Name", &m_gridname, "density");
	param_tooltip(cb, "Name of the grid to lookup");
}

void NodeOpenVDBRead::process()
{
	if (m_filename.empty()) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	openvdb::initialize();

	openvdb::io::File file(m_filename);

	if (!file.open()) {
		setOutputPrimitive("VDB", nullptr);
		std::cerr << "Unable to open file \'" << m_filename << "\'\n";
		return;
	}

	std::cerr << "Grids in file:\n";
	for (auto iter = begin(file), eiter = end(file); iter != eiter; ++iter) {
		std::cerr << *iter << "\n";
	}

	if (m_gridname.size() == 0 || !file.hasGrid(m_gridname.toStdString())) {
		setOutputPrimitive("VDB", nullptr);
		std::cerr << "Cannot lookup \'" << m_gridname.toStdString() << "\' in file.\n";
		return;
	}

	auto grid = file.readGrid(openvdb::Name(m_gridname.toStdString()));

	file.close();

	auto prim = new VDBVolume(grid);
	setOutputPrimitive("VDB", prim);
}

static Node *new_vdbread_node()
{
	return new NodeOpenVDBRead;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_vdbread_node);
}

}
