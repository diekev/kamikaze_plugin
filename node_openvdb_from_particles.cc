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
#include <kamikaze/prim_points.h>

#include <openvdb/tools/ParticlesToLevelSet.h>

#include "volumebase.h"

/* This wrapper class is required by openvdb::tools::ParticlesToLeveSet */
class ParticleList {
	PointList *m_points;
	Attribute *m_scale;
	Attribute *m_velocity;
	bool m_has_radius;
	bool m_has_velocity;
	float m_rad_mult;
	float m_vel_mult;

public:
    // Required by @c openvdb::tools::PointPartitioner
    typedef openvdb::Vec3R  PosType;

    ParticleList(PrimPoints *prim, float rad_mult = 1.0f, float vel_mult = 1.0f)
	    : m_points(prim->points())
	    , m_scale(prim->attribute("scale", ATTR_TYPE_FLOAT))
	    , m_velocity(prim->attribute("velocity", ATTR_TYPE_VEC3))
	    , m_has_radius(m_scale != nullptr)
	    , m_has_velocity(m_velocity != nullptr)
	    , m_rad_mult(rad_mult)
	    , m_vel_mult(vel_mult)
    {}

    /* Do the particles have non-constant radius */
    bool hasRadius() const
	{
		return m_has_radius;
	}

    /* Do the particles have velocity */
    bool hasVelocity() const
	{
		return m_has_velocity;
	}

    // Multiplier for the radius
    float &radiusMult()
	{
		return m_rad_mult;
	}

    const float &radiusMult() const
	{
		return m_vel_mult;
	}

    // The public methods below are the only ones required
    // by tools::ParticlesToLevelSet
    size_t size() const
	{
		return m_points->size();
	}

    // Position of particle in world space
    // This is required by ParticlesToLevelSet::rasterizeSpheres(*this, radius)
    void getPos(size_t n, openvdb::Vec3R& xyz) const
    {
        const auto p = (*m_points)[n];
        xyz[0] = p[0], xyz[1] = p[1], xyz[2] = p[2];
    }

    // Position and radius of particle in world space
    // This is required by ParticlesToLevelSet::rasterizeSpheres(*this)
    void getPosRad(size_t n, openvdb::Vec3R &xyz, openvdb::Real &rad) const
    {
        assert(m_has_radius);

		const auto &p = (*m_points)[n];
        xyz[0] = p[0], xyz[1] = p[1], xyz[2] = p[2];

        rad = m_rad_mult * m_scale->float_(n);
    }

    // Position, radius and velocity of particle in world space
    // This is required by ParticlesToLevelSet::rasterizeTrails
    void getPosRadVel(size_t n, openvdb::Vec3R& xyz,
                      openvdb::Real& rad, openvdb::Vec3R& vel) const
    {
        assert(m_has_velocity);

		const auto p = (*m_points)[n];
        xyz[0] = p[0], xyz[1] = p[1], xyz[2] = p[2];

        rad = m_has_radius ? m_rad_mult * m_scale->float_(n) : m_rad_mult;

		const auto &v = m_velocity->vec3(n);
        vel[0] = m_vel_mult * v[0], vel[1] = m_vel_mult * v[1], vel[2] = m_vel_mult * v[2];
    }

    // Required for attribute transfer
    void getAtt(size_t n, openvdb::Int32& att) const
	{
		att = openvdb::Int32(n);
	}
};

static constexpr auto NODE_NAME = "OpenVDB From Particles";

class NodeOpenVDBFromParticles : public Node {
	bool m_distance_vdb = false;
	QString m_distance_name = "";

	bool m_fog_vdb = false;
	QString m_fog_name = "";

	bool m_mask_vdb = false;
	QString m_mask_name = "";

	float m_bounding_limit = 0.25f;
	float m_voxel_size = 1.0f;
	float m_half_width = 3.0f;

	float m_part_scale = 1.0f;
	float m_min_radius = 1.5f;
	float m_vel_scale = 1.0f;
	float m_trail_res = 1.0f;

	bool m_velocity_trails = false;

public:
	NodeOpenVDBFromParticles();

	void convert(openvdb::FloatGrid::Ptr grid, ParticleList &list);

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeOpenVDBFromParticles::NodeOpenVDBFromParticles()
    : Node(NODE_NAME)
{
	addInput("Points");
	addOutput("VDB");
}

void NodeOpenVDBFromParticles::convert(openvdb::FloatGrid::Ptr grid, ParticleList &list)
{
	openvdb::tools::ParticlesToLevelSet<openvdb::FloatGrid> raster(*grid, nullptr);

    raster.setRmin(m_min_radius);
    raster.setRmax(1e15f);

    if (m_velocity_trails && list.hasVelocity()) {
        raster.rasterizeTrails(list, m_trail_res);
    }
	else if (list.hasRadius()){
        raster.rasterizeSpheres(list);
    }
	else {
        raster.rasterizeSpheres(list, list.radiusMult());
    }

    // always prune to produce a valid narrow-band level set.
    raster.finalize(true);

    if (raster.ignoredParticles()) {
        std::cerr << "Ignored " << raster.getMinCount() << " small and "
		          << raster.getMaxCount() << " large particles (hint: change Minimum Radius in Voxels)";
    }
}

void NodeOpenVDBFromParticles::setUIParams(ParamCallback *cb)
{
	float_param(cb, "Bounding Limit", &m_bounding_limit, 0.0f, 1.0f, m_bounding_limit);
	param_tooltip(cb, "Percentage to increase and decrease the particle radius."
	                  " Used to define the maximum and minimum limit surfaces"
	                  " for the alpha mask construction.");

	float_param(cb, "Voxel Size", &m_voxel_size, 1e-5f, 5.0f, m_voxel_size);
	param_tooltip(cb, "Percentage to increase and decrease the particle radius."
	                  " Used to define the maximum and minimum limit surfaces"
	                  " for the alpha mask construction.");

	/* TODO: toggle for world space units */
	float_param(cb, "Half Width", &m_half_width, 1.0f, 10.0f, m_half_width);
	param_tooltip(cb, "Half the width of the narrow band in voxel units. "
	                  "The default value 3 is recommended for level set volumes.");

	float_param(cb, "Particle Scale", &m_part_scale, 0.0f, 2.0f, m_part_scale);
	param_tooltip(cb, "The point scale attribute, which defines the world space "
	                  "particle radius, will be scaled by this.  A value of one is assumed "
		              "if the scale attribute is missing.");

	float_param(cb, "Minimum Radius", &m_min_radius, 0.0f, 2.0f, m_min_radius);
	param_tooltip(cb, "Minimum radius in voxel units after scaling.  "
	                  "Particles smaller than this limit are ignored.");

	bool_param(cb, "Velocity Trails", &m_velocity_trails, m_velocity_trails);
	param_tooltip(cb, "Velocity trail splatting toggle.  Note this feature "
	                  "requires a velocity point attribute.");

	float_param(cb, "Velocity Scale", &m_vel_scale, 0.0f, 1.0f, m_vel_scale);
	param_tooltip(cb, "Scales the velocity point attribute 'v'.  Use "
	                  "this parameter to control the length of the velocity trails.");

	float_param(cb, "Trail Resolution", &m_trail_res, 0.2f, 2.0f, m_trail_res);
	param_tooltip(cb, "Defines the distance between particle instances.  Use this "
	                  "parameter to control aliasing and number of particle instances.");
}

void NodeOpenVDBFromParticles::process()
{
	auto prim = getInputPrimitive("Points");

	if (!prim) {
		setOutputPrimitive("VDB", nullptr);
		return;
	}

	ParticleList list(static_cast<PrimPoints *>(prim), m_part_scale, m_vel_scale);

	auto grid = openvdb::createLevelSet<openvdb::FloatGrid>(m_voxel_size, m_half_width);

	this->convert(grid, list);

	auto output_prim = new VDBVolume(grid);

	setOutputPrimitive("VDB", output_prim);
}

static Node *new_from_particles_node()
{
	return new NodeOpenVDBFromParticles;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("VDB", NODE_NAME, new_from_particles_node);
}

}
