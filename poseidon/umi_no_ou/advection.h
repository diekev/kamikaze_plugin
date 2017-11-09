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

#include <openvdb/tools/Morphology.h>
#include <openvdb/tools/VolumeAdvect.h>

#include "globals.h"
#include "types.h"
#include "utils.h"

enum {
	ADVECT_SEMI = 0,
	ADVECT_MID,
	ADVECT_RK3,
	ADVECT_RK4,
	ADVECT_MAC,
	ADVECT_BFECC
};

openvdb::tools::Scheme::SemiLagrangian advection_scheme(int scheme)
{
	switch (scheme) {
		default:
		case ADVECT_SEMI:
			return openvdb::tools::Scheme::SEMI;
		case ADVECT_MID:
			return openvdb::tools::Scheme::MID;
		case ADVECT_RK3:
			return openvdb::tools::Scheme::RK3;
		case ADVECT_RK4:
			return openvdb::tools::Scheme::RK4;
		case ADVECT_MAC:
			return openvdb::tools::Scheme::MAC;
		case ADVECT_BFECC:
			return openvdb::tools::Scheme::BFECC;
	}
}

void advect_semi_lagrange(SimulationGlobals &sg, SimulationFields &fields)
{
	Timer(__func__);

	typedef openvdb::tools::Sampler<1, false> Sampler;

	openvdb::tools::VolumeAdvection<VectorGrid, false> advector(*fields.velocity);
	advector.setIntegrator(advection_scheme(sg.advection_scheme));

	sg.max_vel = advector.getMaxVelocity();
	const int n = advector.getMaxDistance(*fields.density, sg.dt);

	if (n > 20) {
		OPENVDB_THROW(openvdb::RuntimeError, "Advection distance is too high!");
	}

	ScalarGrid::Ptr sresult;

	sresult = advector.advect<ScalarGrid, Sampler>(*fields.density, sg.dt);
	fields.density.swap(sresult);

	sresult = advector.advect<ScalarGrid, Sampler>(*fields.temperature, sg.dt);
	fields.temperature.swap(sresult);

	VectorGrid::Ptr result;
	result = advector.advect<VectorGrid, Sampler>(*fields.velocity, sg.dt);
	fields.velocity.swap(result);
}
