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

#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/tools/PointIndexGrid.h>

typedef openvdb::tools::PointIndexGrid PointIndexGrid;

struct Particle {
	size_t index;
	float density;
	float rad;
	openvdb::math::Vec3s vel;
	openvdb::math::Vec3s vel_pic;
	openvdb::math::Vec3s vel_flip;
	openvdb::math::Vec3s vel_t;
	openvdb::math::Vec3s pos;
	openvdb::math::Vec3s pos_t;
	float viscosity;
	float heat;
};

/**
 * This ParticleList class is meant to implement the interfaces required by the
 * following OpenVDB tools:
 *     - openvdb::tools::PointAdvect
 *     - openvdb::tools::PointIndexGrid
 *     - openvdb::tools::PointScatter
 */
class ParticleList {
//    std::vector<Particle *> m_particles;
	size_t m_particle_index;

	std::vector<float> density;
	std::vector<float> rad;
	std::vector<value_type> vel;
	std::vector<value_type> vel_pic;
	std::vector<value_type> vel_flip;
	std::vector<value_type> vel_t;
	std::vector<value_type> m_pos;
	std::vector<value_type> pos_t;
	std::vector<float> viscosity;
	std::vector<float> heat;

public:
    /* Required for bucketing */
    typedef openvdb::math::Vec3s value_type;

    ParticleList()
	    : m_particle_index(0)
    {}

	~ParticleList()
	{
		density.clear();
		rad.clear();
		vel.clear();
		vel_pic.clear();
		vel_flip.clear();
		vel_t.clear();
		m_pos.clear();
		pos_t.clear();
		viscosity.clear();
		heat.clear();
	}

	/**
     * Return the size of the particles array. Always required!
     */
    size_t size() const
    {
        return m_particles.size();
    }

	void reserve(const size_t num)
	{
		//m_particles.reserve(num);

		density.resize(num, 1.0f);
		rad.resize(num, 1.0f);
		vel.resize(num);
		vel_pic.resize(num);
		vel_flip.resize(num);
		vel_t.resize(num);

		/* we will use push_back on pos and pos_t, so just reserve */
		pos.reserve(num);
		pos_t.reserve(num);

		viscosity.resize(num, 1.0f);
		heat.resize(num, 1.0f);
	}

#if 0
    void add(Particle *part)
    {
        m_particles.push_back(part);
    }
#endif

	/**
     * Return the particle at the n'th position in the array.
     */
	Particle *at(const size_t n) const
	{
		return m_particles[n];
	}

	/* Required methods. */

	/**
     * Add a particle at the given pos.
     */
	void add(const value_type &pos)
    {
//		Particle *part = new Particle;

//		part->pos = part->pos_t = pos;
//		part->density = 1.0f;
//		part->index = m_particle_index++;
//		part->vel = part->vel_t = value_type(0.0f);
//		part->rad = 1.0f;

//        m_particles.push_back(part);
		m_pos.push_back(pos);
		pos_t.push_back(pos);
    }

    /**
     * Get the world space position of the n'th particle.
     */
    void getPos(const size_t n, value_type &pos) const
    {
        pos = m_particles[n]->pos;
    }

	value_type &operator[](const size_t n)
	{
		return m_particles[n]->pos;
	}
};

/* Scatters particles in the interior of a level set */
void create_particles(openvdb::FloatGrid::Ptr level_set, ParticleList &particles);

/* Transfer velocity values from the grid to the particles */
void rasterize_particles(ParticleList &particles, PointIndexGrid::Ptr &index_grid, openvdb::Vec3SGrid::Ptr &velocity, openvdb::FloatGrid::Ptr &density);

void advect_particles(ParticleList &particles, openvdb::Vec3SGrid::Ptr velocity, const float dt);

void resample_particles(ParticleList &particles);

void interpolate_pic_flip(openvdb::Vec3SGrid::Ptr &velocity, openvdb::Vec3SGrid::Ptr &velocity_old,
                          ParticleList &particles);
