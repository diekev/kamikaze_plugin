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

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NODE_NAME = "OpenVDB Fill";

enum {
	INDEX = 0,
	WORLD = 1,
	GEOM  = 2,
};

class NodeFill : public VDBNode {
public:
	NodeFill();

	void process() override;
	bool update_properties() override;
};

// Convert a Vec3 value to a vector of another value type or to a scalar value

inline const openvdb::Vec3R &convert_value(const openvdb::Vec3R &val)
{
	return val;
}

// Overload for scalar types (discards all but the first vector component)
template<typename ValueType>
inline typename std::enable_if<!openvdb::VecTraits<ValueType>::IsVec, ValueType>::type
convert_value(const openvdb::Vec3R &val)
{
    return static_cast<ValueType>(val[0]);
}

// Overload for Vec2 types (not currently used)
template<typename ValueType>
inline typename std::enable_if<
    openvdb::VecTraits<ValueType>::IsVec && openvdb::VecTraits<ValueType>::Size == 2,
    ValueType>::type
convert_value(const openvdb::Vec3R &val)
{
    typedef typename openvdb::VecTraits<ValueType>::ElementType ElemType;

    return static_cast<ValueType>(static_cast<ElemType>(val[0]),
	                              static_cast<ElemType>(val[1]));
}

// Overload for Vec3 types
template<typename ValueType>
inline typename std::enable_if<
    openvdb::VecTraits<ValueType>::IsVec && openvdb::VecTraits<ValueType>::Size == 3,
    ValueType>::type
convert_value(const openvdb::Vec3R &val)
{
    typedef typename openvdb::VecTraits<ValueType>::ElementType ElemType;

    return static_cast<ValueType>(static_cast<ElemType>(val[0]),
	                              static_cast<ElemType>(val[1]),
	                              static_cast<ElemType>(val[2]));
}

// Overload for Vec4 types (not currently used)
template<typename ValueType>
inline typename std::enable_if<
    openvdb::VecTraits<ValueType>::IsVec && openvdb::VecTraits<ValueType>::Size == 4,
    ValueType>::type
convert_value(const openvdb::Vec3R &val)
{
    typedef typename openvdb::VecTraits<ValueType>::ElementType ElemType;
    return static_cast<ValueType>(static_cast<ElemType>(val[0]),
	                              static_cast<ElemType>(val[1]),
	                              static_cast<ElemType>(val[2]),
	                              static_cast<ElemType>(1.0));
}

class FillOp {
	const openvdb::math::CoordBBox indexBBox;
    const openvdb::BBoxd worldBBox;
	openvdb::Vec3R value;
	const bool active;

public:
	FillOp(const openvdb::math::CoordBBox &b, const glm::vec3 &val, bool on)
	    : indexBBox(b)
	    , active(on)
    {
		value[0] = val[0];
		value[1] = val[1];
		value[2] = val[2];
	}

    FillOp(const openvdb::BBoxd &b, const glm::vec3 &val, bool on)
	    : worldBBox(b)
	    , active(on)
    {
		value[0] = val[0];
		value[1] = val[1];
		value[2] = val[2];
	}

	template<typename GridType>
    void operator()(GridType &grid) const
    {
        using ValueT = typename GridType::ValueType;

		auto bbox = indexBBox;

        if (worldBBox) {
            openvdb::math::Vec3d imin, imax;
            openvdb::math::calculateBounds(grid.constTransform(),
               worldBBox.min(), worldBBox.max(), imin, imax);

            bbox.reset(openvdb::Coord::floor(imin), openvdb::Coord::ceil(imax));
        }

        grid.fill(bbox, convert_value<ValueT>(value), active);
    }
};

NodeFill::NodeFill()
    : VDBNode(NODE_NAME)
{
	addInput("input");
	addInput("bouding geom (optional)");
	addOutput("output");

	EnumProperty mode_enum;
	mode_enum.insert("Min and Max in Index Space", INDEX);
	mode_enum.insert("Min and Max in World Space", WORLD);
	mode_enum.insert("Reference Geometry", WORLD);

	add_prop("mode", "Bounds", property_type::prop_enum);
	set_prop_enum_values(mode_enum);
	set_prop_tooltip("Index Space:\n"
	                 "    Interpret the given min and max coordinates in index-space units.\n"
	                 "World Space:\n"
	                 "    Interpret the given min and max coordinates in world-space units.\n"
	                 "Reference Geometry:\n"
	                 "    Use the world-space bounds of the reference input geometry.\n");

	add_prop("min", "Min", property_type::prop_vec3);
	set_prop_min_max(-1000.0f, 1000.0f);
	set_prop_default_value_vec3(glm::vec3{ 0.0f, 0.0f, 0.0f });

	add_prop("max", "Max", property_type::prop_vec3);
	set_prop_min_max(-1000.0f, 1000.0f);
	set_prop_default_value_vec3(glm::vec3{ 1.0f, 1.0f, 1.0f });

	add_prop("world_min", "Min", property_type::prop_vec3);
	set_prop_min_max(-10.0f, 10.0f);
	set_prop_default_value_vec3(glm::vec3{ 0.0f, 0.0f, 0.0f });

	add_prop("world_max", "Max", property_type::prop_vec3);
	set_prop_min_max(-10.0f, 10.0f);
	set_prop_default_value_vec3(glm::vec3{ 1.0f, 1.0f, 1.0f });

	add_prop("value", "Value", property_type::prop_vec3);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_tooltip("The value with which to fill voxels\n"
	                 "(y and z are ignored when filling scalar grids)");

	add_prop("activate", "Set Active", property_type::prop_bool);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_tooltip("Mark voxels in the filled region as active");
}

bool NodeFill::update_properties()
{
	auto mode = eval_enum("mode");

	set_prop_visible("min", mode == INDEX);
	set_prop_visible("max", mode == INDEX);

	set_prop_visible("world_min", mode == WORLD);
	set_prop_visible("world_max", mode == WORLD);

	return true;
}

void NodeFill::process()
{
	const auto mode = eval_int("mode");
	const auto value = eval_vec3("value");
	const auto min = eval_vec3("min");
	const auto max = eval_vec3("max");
	const auto world_min = eval_vec3("world_min");
	const auto world_max = eval_vec3("world_max");
	const auto activate = eval_bool("activate");

	std::unique_ptr<FillOp> op;

	switch (mode) {
		case INDEX:
		{
			openvdb::math::Coord co_min((int)min[0], (int)min[1], (int)min[2]);
			openvdb::math::Coord co_max((int)max[0], (int)min[1], (int)min[2]);

			op.reset(new FillOp(openvdb::math::CoordBBox(co_min, co_max), value, activate));
			break;
		}
		case WORLD:
		{
			op.reset(new FillOp(openvdb::BBoxd(&world_min[0], &world_max[0]), value, activate));
			break;
		}
		case GEOM:
		{
			openvdb::BBoxd bbox;
			const auto ref_coll = getInputCollection("bouding geom (optional)");

			if (ref_coll != nullptr) {
				auto iter = primitive_iterator(ref_coll);
				auto prim = iter.get();

				if (prim == nullptr) {
					throw std::runtime_error("No reference geometry found!");
				}

				glm::vec3 geom_min, geom_max;
				prim->computeBBox(geom_min, geom_max);

				bbox.min()[0] = geom_min[0];
				bbox.min()[1] = geom_min[1];
				bbox.min()[2] = geom_min[2];
				bbox.max()[0] = geom_max[0];
				bbox.max()[1] = geom_max[1];
				bbox.max()[2] = geom_max[2];
			}
			else {
				throw std::runtime_error("Reference input is unconnected!");
			}

			op.reset(new FillOp(bbox, value, activate));
			break;
		}
	}

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);
		auto grid = vdb_prim->getGridPtr();

		process_typed_grid(*grid, vdb_prim->storage(), *op);

		vdb_prim->setGrid(grid);
	}
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeFill);
}

}
