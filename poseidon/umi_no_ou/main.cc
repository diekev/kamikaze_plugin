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

#include <openvdb/openvdb.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetUtil.h>

#include "globals.h"

#include "advection.h"
#include "forces.h"
#include "utils.h"

/* Per Bridson, "Fluid Simulation Course Notes", SIGGRAPH 2007:
 * Limit dt to be no greater than 5 grid cells (the largest distance a particle
 * will be able to move during semi-Lagrangian back tracing). Also include forces
 * into the computation to account for velocities induced by them.
 */
void calculate_dt(SimulationGlobals &sg)
{
	const float factor = 1.0f; // user variable
	const float dh = 5.0f * sg.dh;
	const float max_vel = std::max(1e-16f, std::max(dh, sg.max_vel));

	std::cout << "----------max vel = " << sg.max_vel << "\n";
	std::cout << "----------CFL dt = " << dh / std::sqrt(max_vel) << "\n";

	/* dt <= (5dh / max_u) */
	sg.dt = std::min(sg.dt, factor * dh / std::sqrt(max_vel));
}

void create_domain_walls(openvdb::BoolGrid &domain, openvdb::math::BBox<openvdb::Vec3d> &wbbox)
{
	using namespace openvdb;
	using namespace openvdb::math;

	Transform xform = domain.transform();
	CoordBBox bbox = xform.worldToIndexCellCentered(wbbox);

	auto min = bbox.min();
	auto max = bbox.max();

	/* X slabs */
	CoordBBox bbox_xmin(min, Coord(min[0], max[1], max[2]));
	domain.tree().fill(bbox_xmin, true);

	CoordBBox bbox_xmax(Coord(max[0], min[1], min[2]), max);
	domain.tree().fill(bbox_xmax, true);

	/* Y slabs */
	CoordBBox bbox_ymin(min, Coord(max[0], min[1], max[2]));
	domain.tree().fill(bbox_ymin, true);

	CoordBBox bbox_ymax(Coord(min[0], max[1], min[2]), max);
	domain.tree().fill(bbox_ymax, true);

	/* Z slabs */
	CoordBBox bbox_zmin(min, Coord(max[0], max[1], min[2]));
	domain.tree().fill(bbox_zmin, true);

	CoordBBox bbox_zmax(Coord(min[0], min[1], max[2]), max);
	domain.tree().fill(bbox_zmax, true);
}

void add_inflow(const ScalarGrid::Ptr &inflow, ScalarGrid::Ptr &density)
{
	Timer(__func__);

	using namespace openvdb;
	using namespace openvdb::math;

	typedef FloatTree::LeafNodeType LeafType;

	ScalarAccessor dacc = density->getAccessor();

	for (FloatTree::LeafIter lit = inflow->tree().beginLeaf(); lit; ++lit) {
		LeafType &leaf = *lit;

		for (typename LeafType::ValueOnIter it = leaf.beginValueOn(); it; ++it) {
			dacc.setValue(it.getCoord(), 1.0f);
		}
	}
}

void smoke_step(SimulationGlobals &sg, SimulationFields &fields)
{
//	calculate_dt(sg);

	advect_semi_lagrange(sg, fields);

	set_neumann_boundary(*fields.velocity, fields.flags);
	add_buoyancy(sg, fields.velocity, fields.density, fields.temperature, fields.flags);

	solve_pressure(sg, *fields.velocity, *fields.pressure, *fields.flags);
	set_neumann_boundary(*fields.velocity, fields.flags);
}

int main()
{
	using namespace openvdb;
	using namespace openvdb::math;

	std::unique_ptr<SimulationGlobals> sg(new SimulationGlobals);
	sg->dh = 0.1f;
	sg->dt = 0.1f; // 1.0f / 24.0f;
	sg->xform = *Transform::createLinearTransform(sg->dh);
	sg->advection_scheme = ADVECT_RK3;
	sg->max_vel = 0.0f;

	SimulationFields fields;
	initialize_field<FloatGrid>(fields.density, "density", *sg);
	initialize_field<BoolGrid>(fields.obstacle, "obstacle", *sg);
	initialize_field<FloatGrid>(fields.pressure, "pressure", *sg);
	initialize_field<FloatGrid>(fields.temperature, "temperature", *sg);
	initialize_field<VectorGrid>(fields.velocity, "velocity", *sg,
	                             VEC_CONTRAVARIANT_RELATIVE, GRID_STAGGERED);

	BBox<Vec3d> dbbox(Vec3d(-2.5f, -1.0f, -2.5f), Vec3d(2.5f, 8.0f, 2.5f));
	create_domain_walls(*fields.obstacle, dbbox);

	auto inflow = tools::createLevelSetSphere<FloatGrid>(0.75f, Vec3f(0.0f), sg->dh, 3);
	tools::sdfToFogVolume(*inflow);

	fields.velocity->topologyUnion(*inflow);

	int max_frame = 20;
	const float t_frame = 1.0f / 24.0f; // 24 fps, find better default?

	for (int frame(0); frame < max_frame; ++frame) {
		float t = 0.0f;
		std::cout << "====== Step " << frame + 1 << " =======\n";

		if (frame < 10) {
			add_inflow(inflow, fields.density);
			add_inflow(inflow, fields.temperature);
		}

		fields.flags = build_flag_grid(fields.density, fields.obstacle);

		while (t < t_frame) {
			smoke_step(*sg, fields);
			std::cout << "----------dt = " << sg->dt << "\n";
			t = t + sg->dt;
		}

//		fields.velocity->topologyIntersection(*fields.density);

		write_to_disk(fields, frame);
	}
}
