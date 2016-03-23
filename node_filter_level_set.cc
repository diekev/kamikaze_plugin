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

#include "node_filter_level_set.h"

#include <kamikaze/paramfactory.h>

#include <openvdb/tools/LevelSetFilter.h>

#include "levelset.h"

enum {
	LS_FILTER_ACC_FISRT = 0,
	LS_FILTER_ACC_SECOND,
	LS_FILTER_ACC_THIRD,
	LS_FILTER_ACC_WENO5,
	LS_FILTER_ACC_HJWENO5,
};

enum {
	LS_FILTER_MEDIAN = 0,
	LS_FILTER_MEAN,
	LS_FILTER_GAUSSIAN,
	LS_FILTER_MEAN_CURV,
	LS_FILTER_LAPLACIAN,
	LS_FILTER_OFFSET,
};

void NodeFilterLevelSet::setUIParams(ParamCallback *cb)
{
	const char *type_items[] = {
	    "Median", "Mean", "Gaussian", "Mean Curvature", "Laplacian", "Offset",
	    nullptr
	};

	enum_param(cb, "Filter Type", &m_type, type_items, LS_FILTER_ACC_FISRT);

	const char *accuracy_items[] = {
	    "First Bias", "Second Bias", "Third Bias", "WENO5 Bias", "HJ WENO5 Bias",
	    nullptr
	};

	enum_param(cb, "Accuracy", &m_accuracy, accuracy_items, LS_FILTER_ACC_FISRT);

	int_param(cb, "Iteration", &m_iterations, 1, 10, m_iterations);
	int_param(cb, "Width", &m_width, 1, 10, m_width);
	float_param(cb, "Offset", &m_offset, 1.0f, 10.0f, m_offset);
}

void NodeFilterLevelSet::evaluate(Object *ob)
{
	using namespace openvdb;

	auto level_set = static_cast<LevelSet *>(ob);
	auto ls_grid = gridPtrCast<FloatGrid>(level_set->getGridPtr());

	typedef tools::LevelSetFilter<FloatGrid> Filter;

	Filter filter(*ls_grid);

	filter.setTemporalScheme(math::TVD_RK1);

	switch (m_accuracy) {
		case LS_FILTER_ACC_FISRT:   filter.setSpatialScheme(math::FIRST_BIAS);   break;
		case LS_FILTER_ACC_SECOND:  filter.setSpatialScheme(math::SECOND_BIAS);  break;
		case LS_FILTER_ACC_THIRD:   filter.setSpatialScheme(math::THIRD_BIAS);   break;
		case LS_FILTER_ACC_WENO5:   filter.setSpatialScheme(math::WENO5_BIAS);   break;
		case LS_FILTER_ACC_HJWENO5: filter.setSpatialScheme(math::HJWENO5_BIAS); break;
	}

	switch (m_type) {
		case LS_FILTER_MEDIAN:
			for (int i = 0; i < m_iterations; ++i) {
				filter.median(m_width, nullptr);
			}
			break;
		case LS_FILTER_MEAN:
			for (int i = 0; i < m_iterations; ++i) {
				filter.mean(m_width, nullptr);
			}
			break;
		case LS_FILTER_GAUSSIAN:
			for (int i = 0; i < m_iterations; ++i) {
				filter.gaussian(m_width, nullptr);
			}
			break;
		case LS_FILTER_MEAN_CURV:
			for (int i = 0; i < m_iterations; ++i) {
				filter.meanCurvature(nullptr);
			}
			break;
		case LS_FILTER_LAPLACIAN:
			for (int i = 0; i < m_iterations; ++i) {
				filter.laplacian(nullptr);
			}
			break;
		case LS_FILTER_OFFSET:
			for (int i = 0; i < m_iterations; ++i) {
				filter.offset(m_offset, nullptr);
			}
			break;
	}

	level_set->setGrid(ls_grid);
}

static Modifier *new_filter_node()
{
	return new NodeFilterLevelSet;
}

void NodeFilterLevelSet::registerSelf(ModifierFactory *factory)
{
	factory->registerType("Filter Level Set (VDB)", new_filter_node);
}
