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

#include "node_write.h"

#include <kamikaze/paramfactory.h>

#include "levelset.h"

static constexpr auto NODE_NAME = "OpenVDB Write";

NodeWrite::NodeWrite()
    : Node(NODE_NAME)
{
	addInput("Primitive");
	addOutput("Primitive");
}

void NodeWrite::setUIParams(ParamCallback *cb)
{
	file_param(cb, "File Path", &m_filename);
}

void NodeWrite::process()
{
	auto prim = getInputPrimitive("Primitive");

	/* for now just forward the primitive, since kamikaze does not support
	 * multiple output nodes in a giving graph */

	setOutputPrimitive("Primitive", prim);

	if (!prim || m_filename.empty()) {
		return;
	}

	auto ls = static_cast<LevelSet *>(prim);

	openvdb::io::File file(m_filename);
	openvdb::GridCPtrVec grids;

	grids.push_back(ls->getGridPtr()->deepCopyGrid());

	file.write(grids);
}

static Node *new_write_node()
{
	return new NodeWrite;
}

void NodeWrite::registerSelf(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_write_node);
}
