
#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>

#include "particles.h"

void transport_energy(openvdb::FloatGrid::Ptr e_grid,
                      const float alpha,
                      const float beta,
                      const float dt)
{
	using namespace openvdb;
	using namespace openvdb::math;

	assert(alpha >= 0.0f);
	assert(beta >= 0.0f);

	const float num_octaves;

	typedef FloatTree::LeafNodeType LeafType;
	FloatTree::LeafIter leaf_iter = e_grid->tree().beginLeaf();

	typedef tools::GridSampler<FloatGrid::Accessor, tools::PointSampler> LinearFloatSampler;

	LinearFloatSampler sampler(e_grid->getAccessor(), e_grid->transform());

	for (int i = 0; i < num_octaves; ++i) {
		for (; leaf_iter; ++leaf_iter) {
			LeafType &leaf = *leaf_iter;

			for (typename LeafType::ValueOnIter it = leaf.beginValueOn(); it; ++it) {
				Coord co = it.getCoord();

				const float r = sampler.isSample(Coord(co.x() + 1, co.y(), co.z()));
				const float l = sampler.isSample(Coord(co.x() - 1, co.y(), co.z()));
				const float f = sampler.isSample(Coord(co.x(), co.y() + 1, co.z()));
				const float b = sampler.isSample(Coord(co.x(), co.y() - 1, co.z()));
				const float u = sampler.isSample(Coord(co.x(), co.y(), co.z() + 1));
				const float d = sampler.isSample(Coord(co.x(), co.y(), co.z() - 1));

				const float val = sampler.isSample(co);

				float res = val + alpha * dt * (r + l + f + b + u + d - 6.0f * val);
				res = res + beta * dt * (res_prev_octave - res);
			}
		}
	}
}

void advance_net_rotation(openvdb::FloatGrid::Ptr e_grid,
                          openvdb::FloatGrid::Ptr ang_grid,
                          openvdb::FloatGrid::Ptr density,
                          const float delta,
                          const float dt)
{
	using namespace openvdb;
	using namespace openvdb::math;

	assert(alpha >= 0.0f);
	assert(beta >= 0.0f);

	const float num_octaves;
	const float dx = density->transform().voxelSize()[0];

	typedef FloatTree::LeafNodeType LeafType;
	FloatTree::LeafIter leaf_iter = ang_grid->tree().beginLeaf();

	typedef tools::GridSampler<FloatGrid::Accessor, tools::PointSampler> LinearFloatSampler;

	LinearFloatSampler e_sampler(e_grid->getAccessor(), e_grid->transform());
	LinearFloatSampler ang_sampler(ang_grid->getAccessor(), ang_grid->transform());
	LinearFloatSampler dens_sampler(density->getAccessor(), density->transform());

	for (int oct = 0; oct < num_octaves; ++oct) {
		for (; leaf_iter; ++leaf_iter) {
			LeafType &leaf = *leaf_iter;

			for (typename LeafType::ValueOnIter it = leaf.beginValueOn(); it; ++it) {
				Coord co = it.getCoord();

				const float e = e_sampler.isSample(co);
				const float theta = ang_sampler.isSample(co);
				const float dens = dens_sampler.isSample(co);

				const float res = theta + (delta * dt * ((sqrt(2.0f * e / dens)) / (pow(2.0f, -oct) / dx)));

				it.setValue(res);
			}
		}
	}
}

void velocity_predictor(ParticleList &particles, const float dt)
{
	for (size_t i = 0; i < particles.size(); ++i) {
		Particle *p = particles.at(i);

		ParticleList::value_type value = ((4.0f / 3.0f) * p->vel - (1.0f / 3.0f) * p->vel_t + (2.0f / 3.0f) * dt * buoyancy);
	}
}
