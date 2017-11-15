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

#include "types.h"

using namespace openvdb;

constexpr auto M_TAU = 6.28318530717958647692528676655900576839433879875021f;

static float sinus(const float theta)
{
	const float t = (theta / M_TAU) - static_cast<int>(theta);
	return 12.0f * sqrtf(3.0f * t) * (t - 0.5f) * (t - 1.0f);
}

static float cosine(const float theta)
{
	return sinus(theta + M_PI_2);
}

// todo: randomly selected basis vector
void build_basis_vectors(Vec3f vec[128][2])
{
	for (int i = 0; i < 128; ++i) {
		vec[i][0] = Vec3f(1.0f);
		vec[i][1] = Vec3f(1.0f);
	}
}

size_t hash(const size_t s)
{
	size_t h = s;

	h = (h ^ 2747636419) * 2654435769 % (1ul << 32);
	h = (h ^ (s >> 16)) * 2654435769 % (1ul << 32);
	h = (h ^ (s >> 16)) * 2654435769 % (1ul << 32);

	return h;
}

int hash_ijk(const int i, const int j, const int k)
{
	return hash(i ^ hash(j ^ hash(k))) % 128;
}

float interpolate(float noise[8])
{
	return noise[0];
}

float octave_noise(const math::Coord &co, int octave, Vec3f basis[128][2], FloatGrid::Accessor &rotation)
{
	using namespace openvdb::math;

//	const float dx = rotation->transform().voxelSize()[0];
	const float dx = 0.02f;
	const float delta_dx = powf(2.0f, -octave + 1) * dx;

	/* Identify grid’s cube C_ijk. */
	//CoordBBox bbox(co, Coord(co / delta_dx));
	int i = co.x() / delta_dx, j = co.y() / delta_dx, k = co.z() / delta_dx;

	Vec3f pq[8][2] = {
	    { basis[hash_ijk(co.x(), co.y(), co.z())][0], basis[hash_ijk(co.x(), co.y(), co.z())][1] },
	    { basis[hash_ijk(co.x(), co.y(), k     )][0], basis[hash_ijk(co.x(), co.y(), k     )][1] },
	    { basis[hash_ijk(co.x(), j,      co.z())][0], basis[hash_ijk(co.x(), j,      co.z())][1] },
	    { basis[hash_ijk(co.x(), j,      k     )][0], basis[hash_ijk(co.x(), j,      k     )][1] },
	    { basis[hash_ijk(i,      co.y(), co.z())][0], basis[hash_ijk(i,      co.y(), co.z())][1] },
	    { basis[hash_ijk(i,      co.y(), k     )][0], basis[hash_ijk(i,      co.y(), k     )][1] },
	    { basis[hash_ijk(i,      j,      co.z())][0], basis[hash_ijk(i,      j,      co.z())][1] },
	    { basis[hash_ijk(i,      j,      k     )][0], basis[hash_ijk(i,      j,      k     )][1] }
	};

	float cube_rot[8] = {
	    rotation.getValue(Coord(co.x(), co.y(), co.z())),
	    rotation.getValue(Coord(co.x(), co.y(), k     )),
	    rotation.getValue(Coord(co.x(), j,      co.z())),
	    rotation.getValue(Coord(co.x(), j,      k     )),
	    rotation.getValue(Coord(i,      co.y(), co.z())),
	    rotation.getValue(Coord(i,      co.y(), k     )),
	    rotation.getValue(Coord(i,      j,      co.z())),
	    rotation.getValue(Coord(i,      j,      k     )),
	};

	/* Advance gradient vectors. */
	Vec3f gradients[8];

	for (int i = 0; i < 8; ++i) {
		gradients[i] = cosine(cube_rot[i]) * pq[i][0] + sinus(cube_rot[i]) * pq[i][1];
	}

	/* Map gradients to noise values. */
	float noise[8];

	return interpolate(noise);
}

Vec3f potential(math::Coord &co, Vec3f basis[128][2], FloatGrid::Accessor &rotation, FloatGrid::Ptr &energy)
{
	using namespace openvdb::math;

	const float dx = energy->transform().voxelSize()[0];
	const float num_octaves = 3;
	Vec3f psy(0.0f);
	Vec3f psy_b(0.0f);
	float C = 1.0f; // user input
	float amp;

	FloatGrid::Accessor e_grid = energy->getAccessor();
	LinearScalarSampler sampler(energy->getAccessor(), energy->transform());

	for (int b = 1; b <= num_octaves; ++b) {
		psy_b[0] = octave_noise(Coord(co.x(), co.y(), co.z()), b, basis, rotation);
		psy_b[1] = octave_noise(Coord(co.y(), co.z(), co.x()), b, basis, rotation);
		psy_b[2] = octave_noise(Coord(co.z(), co.x(), co.y()), b, basis, rotation);

		// get kinematic density
		const float density = sampler.isSample(co);

		amp = C * pow(2.0f, -b) * dx * sqrtf((2.0f * e_grid.getValue(co) / density)); // todo

		psy = psy + amp * psy_b; // todo
	}

	return psy;
}
