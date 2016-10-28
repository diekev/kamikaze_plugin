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

#include "node_openvdb.h"

#include <kamikaze/prim_points.h>
#include <kamikaze/util_parallel.h>

#include <openvdb/tools/PointPartitioner.h>

#include "volumebase.h"

/* ************************************************************************** */

class VDBPointList {
	PointList *m_points;

public:
	using PosType    = openvdb::Vec3R;
	using ScalarType = PosType::value_type;

    VDBPointList(PointList *list)
	    : m_points(list)
    {}

	const size_t size() const
	{
		if (m_points) {
			return m_points->size();
		}

		return 0ul;
	}

	void getPos(size_t n, PosType &xyz) const
	{
        auto point = (*m_points)[n];
		xyz[0] = point[0];
		xyz[1] = point[1];
		xyz[2] = point[2];
    }
};

/* ************************************************************************** */

class CopyElements {
	PointList *m_points;
	PointList *m_old_points;
	size_t *m_index_array;

public:
    CopyElements(PointList *points, PointList *old_points, size_t *index_array)
        : m_points(points)
	    , m_old_points(old_points)
	    , m_index_array(index_array)
	{}

    void operator()(const tbb::blocked_range<size_t> &range) const
	{
		for (size_t n = range.begin(), N = range.end(); n != N; ++n) {
            (*m_points)[n] = (*m_old_points)[m_index_array[n]];
        }
    }
};

class SetIndices {
public:
    typedef openvdb::tools::UInt32PointPartitioner PointPartitioner;

private:
    const PointPartitioner * const m_partitioner;
    size_t * const m_index_array;

public:

    SetIndices(const PointPartitioner& partitioner, size_t *offsetArray)
	    : m_partitioner(&partitioner)
	    , m_index_array(offsetArray)
	{}

    void operator()(const tbb::blocked_range<size_t> &range) const {

        size_t idx = 0;
        for (size_t n = 0, N = range.begin(); n != N; ++n) {
            idx += m_partitioner->indices(n).size(); // increment to start index
        }

        for (size_t n = range.begin(), N = range.end(); n != N; ++n) {
            for (PointPartitioner::IndexIterator it = m_partitioner->indices(n); it; ++it) {
                m_index_array[idx++] = *it;
            }
        }
    }
};

/* ************************************************************************** */

static constexpr auto NODE_NAME = "OpenVDB Sort Points";

class NodeOpenVDBSortPoints : public VDBNode {

public:
	NodeOpenVDBSortPoints()
	    : VDBNode(NODE_NAME)
	{
		addInput("input");
		addOutput("output");

		add_prop("Bin Size", property_type::prop_float);
		set_prop_default_value_float(1.0f);
		set_prop_min_max(0.0f, 5.0f);
		set_prop_tooltip("The size (length of a side) of the cubic bin, in world units.");
	}

	~NodeOpenVDBSortPoints() = default;

	void process() override
	{
		std::cerr << "Begin process\n";

		PrimitiveCollection tmp_collection(m_collection->factory());

	    const auto bin_size = eval_float("Bin Size");

		std::vector<Primitive *> to_destroy;

		for (auto prim : primitive_iterator(m_collection, PrimPoints::id)) {
			auto point_cloud = static_cast<PrimPoints *>(prim);

			/* Partition points and construct ordered index list. */
			auto transform = openvdb::math::Transform::createLinearTransform(bin_size);

			VDBPointList points(point_cloud->points());

			openvdb::tools::UInt32PointPartitioner partitioner;
			partitioner.construct(points, *transform, /* voxel order */ true);

			const size_t num_points = points.size();
			std::vector<size_t> index_array(num_points);

			parallel_for(tbb::blocked_range<size_t>(0, partitioner.size()),
			             SetIndices(partitioner, index_array.data()));

			/* Create new primitive and order points. */
			auto new_point_cloud = static_cast<PrimPoints *>(point_cloud->copy());
			tmp_collection.add(new_point_cloud);

			parallel_for(tbb::blocked_range<size_t>(0, num_points),
			             CopyElements(new_point_cloud->points(),
			                          point_cloud->points(),
			                          index_array.data()));

			to_destroy.push_back(prim);
		}

		m_collection->destroy(to_destroy);
		m_collection->merge_collection(tmp_collection);
	}
};

/* ************************************************************************** */

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBSortPoints);
}

}
