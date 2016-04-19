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

#pragma once

#include <kamikaze/nodes.h>

class NodeFilterLevelSet : public Node {
	int m_type = 0;
	int m_accuracy = 0;
	int m_iterations = 1;
	int m_width = 1;
	float m_offset = 1.0f;

public:
	NodeFilterLevelSet();
	~NodeFilterLevelSet() = default;

	void setUIParams(ParamCallback *cb) override;
	void process() override;

	static void registerSelf(NodeFactory *factory);
};