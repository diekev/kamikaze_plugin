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

#include "volumebase.h"

#include <openvdb/tools/Clip.h>  /* for tools::clip() */
#include <openvdb/tools/LevelSetUtil.h>  /* for tools::sdfInteriorMask() */

#include "util_openvdb_process.h"

namespace {

struct LevelSetMaskOp {
    template<typename GridType>
    void operator()(const GridType &grid)
    {
        outputGrid = openvdb::tools::sdfInteriorMask(grid);
    }

    openvdb::GridBase::Ptr outputGrid;
};

struct BBoxClipOp {
    BBoxClipOp(const openvdb::BBoxd& bbox_, bool inside_ = true)
	    : bbox(bbox_)
	    , inside(inside_)
    {}

    template<typename GridType>
    void operator()(const GridType &grid)
    {
        outputGrid = openvdb::tools::clip(grid, bbox);
    }

    openvdb::BBoxd bbox;
    openvdb::GridBase::Ptr outputGrid;
    bool inside = true;
};

template<typename GridType>
struct MaskClipDispatchOp {
    MaskClipDispatchOp(const GridType &grid_, bool inside_ = true)
	    : grid(&grid_)
	    , inside(inside_)
    {}

    template<typename MaskGridType>
    void operator()(const MaskGridType& mask)
    {
        outputGrid.reset();

		if (grid) {
			outputGrid = openvdb::tools::clip(*grid, mask);
		}
    }

    const GridType* grid;
    openvdb::GridBase::Ptr outputGrid;
    bool inside = true;
};

struct MaskClipOp {
    MaskClipOp(openvdb::GridBase::ConstPtr mask_, bool inside_ = true)
	    : mask(mask_)
	    , inside(inside_)
    {}

    template<typename GridType>
    void operator()(const GridType &grid)
    {
        outputGrid.reset();

        if (mask) {
            // Dispatch on the mask grid type, now that the source grid type is resolved.
            MaskClipDispatchOp<GridType> op(grid, inside);
            process_typed_grid(*mask, get_grid_storage(*mask), op);
            outputGrid = op.outputGrid;
        }
    }

    openvdb::GridBase::ConstPtr mask;
    openvdb::GridBase::Ptr outputGrid;
    bool inside = true;
};

}  /* unnamed namespace */

static constexpr auto NODE_NAME = "OpenVDB Clip";

class NodeOpenVDBClip : public VDBNode {
public:
	NodeOpenVDBClip()
	    : VDBNode(NODE_NAME)
	{
		addInput("input");
		addInput("mask");
		addOutput("output");

		add_prop("Use Mask", property_type::prop_bool);
		set_prop_default_value_bool(false);
		set_prop_tooltip("If disabled, use the bounding box of the reference geometry\n"
		                 "as the clipping region.");

		add_prop("Keep Inside", property_type::prop_bool);
		set_prop_default_value_bool(true);
		set_prop_tooltip("If enabled, keep voxels that lie inside the clipping region.\n"
		                 "If disabled, keep voxels that lie outside the clipping region.");
	}

	void process() override
	{
		const auto mask_collection = getInputCollection("mask");

		const bool use_mask = eval_bool("Use Mask");
		const bool inside = eval_bool("Keep Inside");

		openvdb::BBoxd bbox;
		openvdb::GridBase::Ptr mask_grid;

		if (mask_collection) {
			VDBVolume *vdb = nullptr;

			for (auto prim : primitive_iterator(mask_collection, VDBVolume::id)) {
				auto volume = static_cast<VDBVolume *>(prim);

				/* Only consider first valid grid. */
				if (volume) {
					vdb = volume;
					break;
				}
			}

			if (use_mask) {
				if (vdb) {
					if (is_level_set(vdb)) {
						// If the mask grid is a level set, extract an interior mask from it.
						LevelSetMaskOp op;
						process_grid_real(vdb->getGrid(), vdb->storage(), op);
						mask_grid = op.outputGrid;
					}
					else {
						mask_grid = vdb->getGridPtr();
					}
				}

				if (!mask_grid) {
					this->add_warning("Mask VDB not found!");
					return;
				}
			}
			else {
				glm::vec3 min, max;

				if (vdb != nullptr) {
					vdb->computeBBox(min, max);
				}
				else {
					min = glm::vec3(0.0f);
					max = glm::vec3(0.0f);
				}

				bbox.min()[0] = min[0];
				bbox.min()[1] = min[1];
				bbox.min()[2] = min[2];
				bbox.max()[0] = max[0];
				bbox.max()[1] = max[1];
				bbox.max()[2] = max[2];
			}
		}

		auto num_level_sets = 0;
		for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
			auto vdb = static_cast<VDBVolume *>(prim);

			if (is_level_set(vdb)) {
				++num_level_sets;
			}

			openvdb::GridBase::Ptr outGrid;

			if (mask_grid) {
				MaskClipOp op(mask_grid, inside);
				process_typed_grid(vdb->getGrid(), vdb->storage(), op);
				outGrid = op.outputGrid;
			}
			else {
				BBoxClipOp op(bbox, inside);
				process_typed_grid(vdb->getGrid(), vdb->storage(), op);
				outGrid = op.outputGrid;
			}

			/* TODO: find a way to replace a primitive with an another one. */
			build_vdb_prim(m_collection, outGrid);
		}

		if (num_level_sets > 0) {
			if (num_level_sets == 1) {
				this->add_warning("A level set grid was clipped, the"
				                  " resulting grid might not be a valid"
				                  " level set.\n");
			}
			else {
				this->add_warning("Some level sets were clipped, the"
				                  " resulting grids might not be a valid"
				                  " level sets.\n");
			}
		}
	}
};

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBClip);
}

}
