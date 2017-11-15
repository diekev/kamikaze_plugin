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
#include <openvdb/tools/LevelSetUtil.h>

#include <fstream>

#include "types.h"

struct SimulationGlobals {
	float dx;  // voxel size
	float dt;  // time step
	openvdb::math::Transform xform;
};

using FlagGrid = openvdb::Int32Grid;

void create_domain_walls(FlagGrid &domain, openvdb::math::BBox<openvdb::Vec3d> &wbbox)
{
	using namespace openvdb;
	using namespace openvdb::math;

	Transform xform = domain.transform();
	CoordBBox bbox = xform.worldToIndexCellCentered(wbbox);

	/* X slabs */
	CoordBBox bbox_xmin(bbox.min(), Coord(bbox.min()[0], bbox.max()[1], bbox.max()[2]));
	domain.tree().fill(bbox_xmin, TypeObstacle);

	CoordBBox bbox_xmax(Coord(bbox.max()[0], bbox.min()[1], bbox.min()[2]), bbox.max());
	domain.tree().fill(bbox_xmax, TypeObstacle);

	/* Y slabs */
	CoordBBox bbox_ymin(bbox.min(), Coord(bbox.max()[0], bbox.min()[1], bbox.max()[2]));
	domain.tree().fill(bbox_ymin, TypeObstacle);

	CoordBBox bbox_ymax(Coord(bbox.min()[0], bbox.max()[1], bbox.min()[2]), bbox.max());
	domain.tree().fill(bbox_ymax, TypeObstacle);

	/* Z slabs */
	CoordBBox bbox_zmin(bbox.min(), Coord(bbox.max()[0], bbox.max()[1], bbox.min()[2]));
	domain.tree().fill(bbox_zmin, TypeObstacle);

	CoordBBox bbox_zmax(Coord(bbox.min()[0], bbox.min()[1], bbox.max()[2]), bbox.max());
	domain.tree().fill(bbox_zmax, TypeObstacle);
}

int main()
{
	using namespace openvdb;
	using namespace openvdb::math;

//	std::unique_ptr<SimulationGlobals> sg(new SimulationGlobals);
//	sg->dx = 0.1f;
//	sg->dt = 1.0f / 24.0f;
//	sg->xform = *Transform::createLinearTransform(sg->dx);

	BBox<Vec3d> dbbox(Vec3d(-4.0f, -3.0f, -3.0f), Vec3d(4.0f, 3.0f, 3.0f));

	FlagGrid::Ptr domain = FlagGrid::create(0);
	domain->transform() = sg->xform;
	domain->setName("domain");
	create_domain_walls(*domain, dbbox);

	BBox<Vec3f> fbbox(Vec3f(-3.8f, -2.8f, -2.8f), Vec3f(-2.5f, 2.8f, 2.8f));
	auto inflow = tools::createLevelSetBox<FloatGrid>(fbbox, sg->xform);
	inflow->setName("inflow");

//	io::File file("/home/kevin/poseidon_domain.vdb");
//	GridPtrVec grids;
//	grids.push_back(domain->deepCopy());
//	grids.push_back(inflow->deepCopy());
//	file.write(grids);
}

#if 0
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/LevelSetSphere.h>

#include "grid_ops.h"
#include "forces.h"
#include "particles.h"
#include "solvers.h"

using namespace openvdb;
int main()
{
	using namespace openvdb;
	using namespace openvdb::math;

	Timer("Simulation Time");

//	initialize();

//	io::File lsfile("/home/kevin/src/test_files/openvdb/bunny.vdb");
//	lsfile.open();

//	auto level_set = gridPtrCast<FloatGrid>(lsfile.readGrid("ls_bunny"));

	auto level_set = tools::createLevelSetSphere<FloatGrid>(1.0f, Vec3f(0.0f), 0.1f, 4.0f);
	level_set->setName("density");

	auto dt = 0.1f;
	auto iteration = 5;
	auto count = 0;

	ParticleList particles;
	create_particles(level_set, particles);
	auto index_grid = tools::createPointIndexGrid<PointIndexGrid>(particles, level_set->transform());

	auto velocity = Vec3SGrid::create(Vec3s(0.0f));
	velocity->setTransform(level_set->transformPtr());
	velocity->setVectorType(VEC_CONTRAVARIANT_RELATIVE);
	velocity->setGridClass(GRID_STAGGERED);

	auto density = FloatGrid::create(0.0f);
	density->setTransform(level_set->transformPtr());
	velocity->setVectorType(VEC_INVARIANT);

	std::cout << "Number of particles: " << particles.size() << "\n";

//	lsfile.close();
	GridPtrVec grids;

	while (iteration--) {
		printf("== Step %d ==\n", ++count);
		auto start_dt = time_dt();

		// 2. Transfer particle velocities to a staggered grid.
		rasterize_particles(particles, index_grid, velocity, density);

		// 3. FLIP: Save a copy of the grid velocities
		Vec3SGrid::Ptr velocity_old = velocity->deepCopy();

		// 4. Calculate and apply external forces.
		add_force(velocity, density, dt);

		// 7. Calculate the pseudo-pressure gradient using a Preconditioned
		// Conjugate Gradient method
		project(velocity);

		interpolate_pic_flip(velocity, velocity_old, particles);

		// 11. Update the particle positions
		advect_particles(particles, velocity, dt);
		index_grid = tools::getValidPointIndexGrid<PointIndexGrid>(particles, index_grid);

		resample_particles(particles);

		CoordBBox bbox = velocity->evalActiveVoxelBoundingBox();

		assert(!math::isExactlyEqual(bbox.min(), Coord(std::numeric_limits<int>::min())));
		assert(!math::isExactlyEqual(bbox.max(), Coord(std::numeric_limits<int>::max())));

		velocity->setName(get_name_for_frame("velocity", count));
		density->setName(get_name_for_frame("density", count));
		grids.push_back(velocity->deepCopy());
		grids.push_back(density->deepCopy());

		printf("Total Time: %fs\n", time_dt() - start_dt);
		printf("============================================================\n");
	}

	io::File file("/home/kevin/poseidon.vdb");

	file.setCompression(io::COMPRESS_ZIP);
	file.write(grids);
}

#include <random>

#include "utils.h"

template <typename GridType, typename ValueType>
typename GridType::Ptr create_field(const ValueType background,
                                    const openvdb::math::Transform &xform,
                                    const std::string &name,
                                    const openvdb::VecType vector_type = openvdb::VEC_INVARIANT)
{
	typename GridType::Ptr field = GridType::create(background);
	field->setTransform(xform.copy());
	field->setName(name);
	field->setVectorType(vector_type);

	return field;
}

int main()
{
	using namespace openvdb;
	using namespace openvdb::math;

	Timer("Simulation Time");

	/* every field in this simulator share the same tranform */
	Transform::Ptr xform = Transform::createLinearTransform(0.5f);

	/* create collision field */
	FloatGrid::Ptr collision = create_field<FloatGrid>(0.0f, *xform, "collision");

	/* create domain object, note: CoordBBox is inclusive */
	CoordBBox domain(Coord(0), Coord(99, 99, 399));
	collision->fill(domain, 0.0f);

	std::mt19937 rng(28071991);
	std::uniform_real_distribution<float> dist(0.0f, 1.0f);

	Coord ijk;
	int &i = ijk[0], &j = ijk[1], &k = ijk[2];

	FloatGrid::Accessor coll_mask = collision->getAccessor();

	for (i = 0; i < 100; ++i) {
		for (j = 0; j < 100; ++j) {
			for (k = 0; k < 400; ++k) {
				coll_mask.setValue(ijk, dist(rng));
			}
		}
	}

	GridPtrVec grids;
	grids.push_back(collision);
	io::File file("/home/kevin/collision.vdb");

	file.setCompression(io::COMPRESS_ZIP);
	file.write(grids);
}

#include <openvdb/io/Queue.h>

struct NotificationQueue {
	// Use a concurrent container, because queue callback functions
	// must be thread-safe.
	typedef tbb::concurrent_hash_map<openvdb::io::Queue::Id, std::string> FilenameMap;
	FilenameMap filenames;

	// Callback function that prints the status of a completed task.
	void callback(openvdb::io::Queue::Id id, openvdb::io::Queue::Status status)
	{
		const bool ok = (status == openvdb::io::Queue::SUCCEEDED);
		FilenameMap::accessor acc;
		if (filenames.find(acc, id)) {
			std::cout << (ok ? "wrote " : "failed to write ")
			          << acc->second << std::endl;
			filenames.erase(acc);
		}
	}
};

void write_grids_to_file(NotificationQueue &notifier,
                         openvdb::io::Queue &queue,
                         const openvdb::GridCPtrVec &grids,
                         const std::string &name)
{
	using namespace openvdb::io;

	Queue::Id id = queue.write(grids, File(name));

	// Associate the filename with the ID of the queued task.
	NotificationQueue::FilenameMap::accessor acc;
	notifier.filenames.insert(acc, id);
	acc->second = filename;
}

int main()
{
	using namespace openvdb;
	using namespace openvdb::io;

	// Construct an object to receive notifications from the queue.
	// The object's lifetime must exceed the queue's.
	NotificationQueue notifier;
	Queue queue;

	// Register the callback() method of the MyNotifier object
	// to receive notifications of completed tasks.
	queue.addNotifier(boost::bind(&NotificationQueue::callback, &notifier, _1, _2));

	// Queue grids for output (e.g., for each step of a simulation).
	for (int step = 1; step <= 10; ++step) {
		openvdb::FloatGrid::Ptr grid = FloatGrid::create(0.0f);

		std::ostringstream os;
		os << "mygrid." << step << ".vdb";
		const std::string filename = os.str();

		write_grids_to_file(notifier, queue, grid, filename);
	}

	std::cout << "End main\n";
}
#endif
