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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2015 Kévin Dietrich.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "forces.h"

#include <openvdb/tools/ValueTransformer.h>

#include "particles.h"
#include "utils.h"

#if 0
void add_forces(ParticleList &particles, const float dt)
{
	Timer(__func__);
	using namespace openvdb;

	const float rise = 0.1f;
	const float fall = 0.1f;
	const float smk_tmp = 1.0f;
	const float tmp_amb = 0.0f;
	math::Vec3s gravity(0.0f, 0.0f, -9.80665f);
	//gravity /= gravity.length();

	tbb::parallel_for(tbb::blocked_range<size_t>(0, particles.size()),
					  [&](const tbb::blocked_range<size_t>& r)
	{
		for (size_t i = r.begin(); i != r.end(); ++i) {
			const float buoyancy = rise * (smk_tmp - tmp_amb) + fall * particles.at(i)->density;

			particles.at(i)->vel -= (gravity * buoyancy * dt);
		}
	});
}

/* Add forces to a velocity grid */
struct AddForcesOp {
	openvdb::Vec3SGrid &m_grid;
	float m_dt;

	typedef openvdb::Vec3STree::LeafNodeType LeafType;


	AddForcesOp(openvdb::Vec3SGrid &grid, const float dt)
	    : m_grid(grid)
	    , m_dt(dt)
	{}

	inline void operator()(const openvdb::Vec3STree::LeafIter &lit) const
	{
		using namespace openvdb;
		using namespace openvdb::math;
		LeafType &leaf = *lit;

		const float rise = 0.1f;
		const float fall = 0.1f;
		const float smk_tmp = 1.0f;
		const float tmp_amb = 0.0f;
		math::Vec3s gravity(0.0f, 0.0f, -9.80665f);
		const float buoyancy = rise * (smk_tmp - tmp_amb) + fall * 1.0f;

//		Vec3SGrid::Accessor acc = m_grid.getAccessor();
//		const float dx = m_grid.transform().voxelSize()[0];

		for (typename LeafType::ValueOnIter it = leaf.beginValueOn(); it; ++it) {
			it.setValue(it.getValue() * (buoyancy * -gravity * m_dt));
			//acc.modifyValue(it.getCoord(), [&](math::Vec3s val) { val += (buoyancy * -gravity * m_dt); });
		}
	}
};
#endif

void add_force(openvdb::Vec3SGrid::Ptr velocity, openvdb::FloatGrid::Ptr density, const float dt)
{
	using namespace openvdb;
	using namespace openvdb::math;

	typedef openvdb::Vec3STree::LeafNodeType LeafType;

	Timer(__func__);

	Vec3SGrid::Accessor vacc = velocity->getAccessor();
	FloatGrid::Accessor dacc = density->getAccessor();

	for (openvdb::Vec3STree::LeafIter lit = velocity->tree().beginLeaf(); lit; ++lit) {
		const float rise = -0.001f;
		const float fall = 0.1f;
		const float smk_tmp = 1.0f;
		const float tmp_amb = 0.0f;
		math::Vec3s gravity(0.0f, 0.0f, -9.80665f);

		LeafType &leaf = *lit;

		auto rise_coef = rise * (smk_tmp - tmp_amb);
		for (typename LeafType::ValueOnIter it = leaf.beginValueOn(); it; ++it) {
			Coord co(it.getCoord());
			const float buoyancy = rise_coef + fall * dacc.getValue(co);

			vacc.setValue(co, vacc.getValue(co) + buoyancy * -gravity * dt);
		}
	}
}

/* From "FLIP Smoke Simulation and Rendering with IBL", page 2, 3.1.
 *
 * note: decay_rate should be user defined
 */
void update_temperature(std::vector<float> &temperature, const float decay_rate, const float dt)
{
	const float scale = std::pow(M_E, -decay_rate * dt);

	tbb::parallel_for(tbb::blocked_range<size_t>(0, temperature.size()),
	                  [&](const tbb::blocked_range &r)
	{
		for (size_t i(r.begin()), e(r.end()); i != e; ++i) {
			temperature[i] *= scale;
		}
	});
}

#include <openvdb/math/Stencils.h>

void check_collision(ParticleList &particles, const openvdb::FloatGrid &obstacle)
{
	using namespace openvdb;

	FloatGrid::Accessor acc = obstacle.getAccessor();
	math::GradStencil<FloatGrid> stencil(obstacle);

	for (size_t i(0); i < particles.size(); ++i) {
		Particle *p = particles.at(i);
		Vec3f pos = p->pos;
		math::Coord co = obstacle.transform().worldToIndexCellCentered(pos);

		float phi = acc.getValue(co);

		/* check whether the particles is outside the level set */
		if (phi < 0.0f) {
			stencil.moveTo(co);
			//Vec3f grad = stencil.gradient();
			// project onto surface point X_s, X_s = X - phi(X)*grad(phi(X))
			p->pos = stencil.cpt();
		}
	}
}
