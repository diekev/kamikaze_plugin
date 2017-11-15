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

#include <cstdio>
#include <string>

/* return current time */
double time_dt();

/* A utility class to print the time elapsed during its lifetime, usefull for e.g.
 * timing the overall execution time of a function */
class ScopeTimer {
	double m_start;
	std::string m_message;

public:
	ScopeTimer(const std::string &message)
	    : m_message(message)
	{
		m_start = time_dt();
	}

	~ScopeTimer()
	{
		printf("%s: %fs\n", m_message.c_str(), time_dt() - m_start);
	}
};

#define Timer(x) \
	TimeFunc func(x);

/* generate a name containing the frame number */
std::string get_name_for_frame(const std::string &name, const int frame);

#if 0

struct SmokeFields {
	openvdb::FloatGrid::Ptr density;
	openvdb::Vec3SGrid::Ptr velocity;
	openvdb::Vec3SGrid::Ptr velocity_old;
};

FloatGrid::Ptr createSphere(float voxel_size)
{
	auto sphere = tools::createLevelSetSphere<FloatGrid>(2.0f, Vec3f(0.0f), voxel_size, 4.0f);
	tools::sdfToFogVolume(*sphere);

	return sphere;
}
#endif
