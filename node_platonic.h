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

enum {
	PLATONIC_SPHERE = 0,
	PLATONIC_CUBE   = 1,
};

class NodePlatonic : public Node {
	float voxel_size = 0.1f;
	float half_width = 3.0f;
	float radius = 2.0f;
	float center[3] = { 0.0f, 0.0f, 0.0f };
	int type = PLATONIC_SPHERE;

public:
	NodePlatonic();
	~NodePlatonic() = default;

	void setUIParams(ParamCallback *cb) override;
	void process() override;

	static void registerSelf(NodeFactory *factory);
};
