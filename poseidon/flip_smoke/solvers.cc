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

#include "solvers.h"

#include <openvdb/math/FiniteDifference.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/Morphology.h> // for tools::erodeVoxels()
#include <openvdb/tools/PoissonSolver.h>

#include "grid_ops.h"
#include "particles.h"
#include "utils.h"

pcg::State SmokeData::solve_pressure_equation(const VectorGrid &u,
                                              const ScalarGrid &mask_fluid,
                                              const ScalarGrid &mask_solid,
                                              float bg_pressure,
                                              ScalarGrid &q)
{
	pcg::State result;
	result.success = false;
	result.iterations = 0;
	result.absoluteError = 0.0f;
	result.relativeError = 0.0f;

	ScalarGrid::Ptr div_u = calc_divergence();

	VIndexTree::Ptr index_tree = tools::poisson::createIndexTree(div_u->tree());
	VectorType::Ptr b = tools::poisson::createVectorFromTree<float>(div_u->tree(), *index_tree);

	const pcg::SizeType rows = b->size();
	MatrixType A(rows);

	ScalarGrid::ConstAccessor acc_solid(mask_solid.tree());
	ScalarGrid::ConstAccessor acc_fluid(mask_fluid.tree());

//	float scale = 1.0f/cell_size();
	float scale = 1.0f;

	for (ScalarGrid::ValueOnCIter it = div_u->cbeginValueOn(); it; ++it) {
		Coord c = it.getCoord();
		VIndex irow = index_tree->getValue(c);

		// TODO probably this can be optimized significantly
		// by shifting grids as a whole and encode neighbors
		// as bit flags or so ...
		// XXX look for openvdb stencils? div operator works similar?

		Coord neighbors[6] = {
		    Coord(c[0]-1, c[1], c[2]),
		    Coord(c[0]+1, c[1], c[2]),
		    Coord(c[0], c[1]-1, c[2]),
		    Coord(c[0], c[1]+1, c[2]),
		    Coord(c[0], c[1], c[2]-1),
		    Coord(c[0], c[1], c[2]+1)
		};

		float diag = 0.0f;
		float bg = 0.0f;
		for (int i = 0; i < 6; ++i) {
			const Coord &nc = neighbors[i];

			const bool is_solid = acc_solid.isValueOn(nc);
			const bool is_fluid = acc_fluid.isValueOn(nc);
			const bool is_empty = !is_solid && !is_fluid;

			/* add matrix entries for interacting cells (non-solid neighbors) */
			if (!is_solid) {
				diag -= 1.0f;
			}

			if (is_fluid) {
				VIndex icol = index_tree->getValue(nc);
				if (icol != VINDEX_INVALID) {
					A.setValue(irow, icol, 1.0f);
				}
			}

			/* add background pressure terms */
			if (is_empty) {
				bg -= bg_pressure;
			}
		}

		/* XXX degenerate case (only solid neighbors), how to handle? */
		if (diag == 0.0f)
			diag = 1.0f;

		A.setValue(irow, irow, diag * scale);
		(*b)[irow] += bg;
	}
	assert(A.isFinite());

	/* preconditioner for faster convergence */
	pcg::JacobiPreconditioner<MatrixType> precond(A);

	/* solve A * x = B for x */
	MatrixType::VectorType x(rows, 0.0f);

	pcg::State terminator = pcg::terminationDefaults<float>();
	terminator.iterations = 100;
	terminator.relativeError = terminator.absoluteError = 1.0e-4;

	util::NullInterrupter interrupter;
	result = math::pcg::solve(A, *b, x, precond, interrupter, terminator);

	if (result.success) {
		q.setTree(tools::poisson::createTreeFromVector<float>(x, *index_tree, 0.0f));
	}
	else {
		q.clear();
	}
//	mul_grid_fl(q, cell_size());
	mul_grid_fl(q, scale);

	return result;
}

void project(openvdb::Vec3SGrid::Ptr &velocity)
{
	using namespace openvdb;
	using namespace openvdb::math;

	Timer(__func__);

	const float dx = velocity->transform().voxelSize()[0];

	/* compute divergence of the velocity field */
	FloatGrid::Ptr div = tools::divergence(*velocity);

	/* solve poisson equation */

	/* open at the top */
	auto boundary = [](const math::Coord &ijk, const math::Coord &neighbor,
	                double &source, double &diagonal)
	{
		if (neighbor.x() == ijk.x() && neighbor.y() == ijk.y()) {
			(neighbor.z() < ijk.z()) ? source : diagonal -= 1.0;
		}
	};

	util::NullInterrupter interrupt;
	math::pcg::State state = math::pcg::terminationDefaults<float>();

	FloatTree::Ptr solution = tools::poisson::solveWithBoundaryConditions(div->tree(), boundary, state, interrupt);

	FloatGrid::Ptr pressure = FloatGrid::create(solution);

	typedef openvdb::Vec3STree::LeafNodeType LeafType;
	Vec3SGrid::Accessor vel_grid = velocity->getAccessor();
	FloatGrid::Accessor pre_grid = pressure->getAccessor();

	/* Project out solution. */
	for (openvdb::Vec3STree::LeafIter lit = velocity->tree().beginLeaf(); lit; ++lit) {
		LeafType &leaf = *lit;

		for (typename LeafType::ValueOnIter it = leaf.beginValueOn(); it; ++it) {
			Coord ijk = it.getCoord();

			// already divided by 2
			Vec3s pre = Vec3s(D1<CD_2ND>::inX(pre_grid, ijk),
			                  D1<CD_2ND>::inY(pre_grid, ijk),
			                  D1<CD_2ND>::inZ(pre_grid, ijk));

			Vec3s vel = vel_grid.getValue(ijk) - pre * dx;

			vel_grid.setValue(ijk, vel);
		}
	}
}
