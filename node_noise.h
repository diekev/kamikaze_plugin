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

#include <kamikaze/modifiers.h>

class NodeNoise : public Modifier {
	int m_octaves = 1;
    float m_frequency = 1.0f;
    float m_amplitude = 1.0f;
    float m_persistence = 1.0f;
    float m_lacunarity = 2.0f;

public:
	NodeNoise() = default;

	float evalNoise(float x, float y, float z);
	void setUIParams(ParamCallback *cb) override;
	void evaluate(Object *ob) override;

	static void registerSelf(ModifierFactory *factory);
};
