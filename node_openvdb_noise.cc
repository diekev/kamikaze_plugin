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
#include <kamikaze/noise.h>
#include <kamikaze/paramfactory.h>

#include <openvdb/math/Operators.h>

#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Noise";

class NodeNoise : public Node {
	int m_octaves = 1;
    float m_frequency = 1.0f;
    float m_amplitude = 1.0f;
    float m_persistence = 1.0f;
    float m_lacunarity = 2.0f;

public:
	NodeNoise();

	float evalNoise(float x, float y, float z);
	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeNoise::NodeNoise()
    : Node(NODE_NAME)
{
	addInput("VDB");
	addOutput("VDB");
}

float NodeNoise::evalNoise(float x, float y, float z)
{
	float output = 0.0f;
	float frequency = m_frequency;
	float amplitude = m_amplitude;

	for (size_t i = 0; i < m_octaves; i++) {
		output += (amplitude * simplex_noise_3d(x * frequency, y * frequency, z * frequency));

		frequency *= m_lacunarity;
		amplitude *= m_persistence;
	}

	return output;
}

void NodeNoise::setUIParams(ParamCallback *cb)
{
	int_param(cb, "Octaves", &m_octaves, 1, 10, m_octaves);
	float_param(cb, "Frequency", &m_frequency, 0.0f, 1.0f, m_frequency);
	float_param(cb, "Amplitude", &m_amplitude, 0.0f, 10.0f, m_amplitude);
	float_param(cb, "Persistence", &m_persistence, 0.0f, 10.0f, m_persistence);
	float_param(cb, "Lacunarity", &m_lacunarity, 0.0f, 10.0f, m_lacunarity);
}

void NodeNoise::process()
{
	auto prim = getInputPrimitive("VDB");

	if (!prim) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	auto vdb_prim = static_cast<VDBVolume *>(prim);

	if (!is_level_set(vdb_prim)) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	using CPT = openvdb::math::CPT<openvdb::math::GenericMap, openvdb::math::CD_2ND>;
    using StencilType = openvdb::math::SecondOrderDenseStencil<openvdb::FloatGrid>;

	auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(vdb_prim->getGridPtr());

	StencilType stencil(*grid);  /* uses its own grid accessor */

	const auto &xform = grid->transform();
	openvdb::Vec3R worldPt;  /* world coordinates  */
	openvdb::math::GenericMap map(*grid);
	float noise;

	init_perm_table();

	for (auto iter = grid->beginValueOn(); iter; ++iter) {
		stencil.moveTo(iter);
        worldPt = xform.indexToWorld(CPT::result(map, stencil));

        noise = evalNoise(worldPt.x(), worldPt.y(), worldPt.z());

        iter.setValue(*iter + noise);
	}

	vdb_prim->setGrid(grid);

	setOutputPrimitive("VDB", vdb_prim);
}

static Node *new_noise_node()
{
	return new NodeNoise;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_noise_node);
}

}
