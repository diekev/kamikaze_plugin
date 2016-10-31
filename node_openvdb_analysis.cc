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

#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/util/NullInterrupter.h>

#include "util_openvdb_process.h"
#include "volumebase.h"

/* ************************************************************************** */

namespace {

template<template<typename GridT, typename MaskType, typename InterruptT> class ToolT>
struct ToolOp
{
    ToolOp(bool t, openvdb::util::NullInterrupter& boss, const openvdb::BoolGrid *mask = NULL)
        : mMaskGrid(mask)
        , mThreaded(t)
        , mBoss(boss)
    {
    }

    template<typename GridType>
    void operator()(const GridType& inGrid)
    {
        if (mMaskGrid) {
            // match transform
            openvdb::BoolGrid regionMask;
            regionMask.setTransform(inGrid.transform().copy());
            openvdb::tools::resampleToMatch<openvdb::tools::PointSampler>(*mMaskGrid, regionMask, mBoss);

            ToolT<GridType, openvdb::BoolGrid, openvdb::util::NullInterrupter> tool(inGrid, regionMask, &mBoss);
            mOutGrid = tool.process(mThreaded);

        }
		else {
            ToolT<GridType, openvdb::BoolGrid/*dummy*/, openvdb::util::NullInterrupter> tool(inGrid, &mBoss);
            mOutGrid = tool.process(mThreaded);
        }
    }

    const openvdb::BoolGrid *mMaskGrid;
    openvdb::GridBase::Ptr  mOutGrid;
    bool                    mThreaded;
    openvdb::util::NullInterrupter&      mBoss;
};


struct MaskOp {
    template<typename GridType>
    void operator()(const GridType& grid)
    {
        if (openvdb::GRID_LEVEL_SET == grid.getGridClass()) {
            mMaskGrid = openvdb::tools::sdfInteriorMask(grid);
        }
		else {
            mMaskGrid = openvdb::BoolGrid::create(false);
            mMaskGrid->setTransform(grid.transform().copy());
            mMaskGrid->tree().topologyUnion(grid.tree());
        }
    }

    openvdb::BoolGrid::Ptr mMaskGrid;
};

} // unnamed namespace

/* ************************************************************************** */

enum {
    OP_GRADIENT   = 0,
    OP_CURVATURE  = 1,
    OP_LAPLACIAN  = 2,
    OP_CPT        = 3,
    OP_DIVERGENCE = 4,
    OP_CURL       = 5,
    OP_MAGNITUDE  = 6,
    OP_NORMALIZE  = 7
};

static constexpr auto NODE_NAME = "OpenVDB Analysis";

class NodeOpenVDBAnalysis : public VDBNode {
	static const char *operator_name[];

public:
	NodeOpenVDBAnalysis();

	bool update_properties() override;

	void process() override;
};

const char *NodeOpenVDBAnalysis::operator_name[] = {
    "gradient",
    "curvature",
    "laplacian",
    "closest point transform",
    "divergence",
    "curl",
    "magnitude",
    "normalize"
};

NodeOpenVDBAnalysis::NodeOpenVDBAnalysis()
    : VDBNode(NODE_NAME)
{
	addInput("input");
	addInput("mask VDB (optional)");
	addOutput("output");

	/* Operator. */
    {
		EnumProperty items;
		items.insert("Gradient       (Scalar->Vector)", OP_GRADIENT);
		items.insert("Curvature      (Scalar->Scalar)", OP_CURVATURE);
		items.insert("Laplacian      (Scalar->Scalar)", OP_LAPLACIAN);
		items.insert("Closest Point  (Scalar->Vector)", OP_CPT);
		items.insert("Divergence     (Vector->Scalar)", OP_DIVERGENCE);
		items.insert("Curl           (Vector->Vector)", OP_CURL);
		items.insert("Length         (Vector->Scalar)", OP_MAGNITUDE);
		items.insert("Normalize      (Vector->Vector)", OP_NORMALIZE);

		add_prop("operator", "Operator", property_type::prop_enum);
		set_prop_enum_values(items);
		set_prop_default_value_int(0);
    }

	/* Output name. */
    {
		EnumProperty items;
		items.insert("Keep Incoming VDB Names", 0);
		items.insert("Append Operation Name", 1);
		items.insert("Custom Name", 2);

		add_prop("outputName", "Output Name", property_type::prop_enum);
		set_prop_enum_values(items);
		set_prop_default_value_int(0);
		set_prop_tooltip("Rename output grid(s)");
    }

	add_prop("customName", "Custom Name", property_type::prop_string);
	set_prop_tooltip("Renames all output grids with this custom name");
}

bool NodeOpenVDBAnalysis::update_properties()
{
    bool useCustomName = (eval_enum("outputName") == 2);
    set_prop_visible("customName", useCustomName);

	return true;
}

void NodeOpenVDBAnalysis::process()
{
    const int whichOp = eval_enum("operator");
    if (whichOp < OP_GRADIENT || whichOp > OP_NORMALIZE) {
        std::ostringstream ostr;
        ostr << "expected OP_GRADIENT <= operator <= OP_NORMALIZE, got " << whichOp;
        throw std::runtime_error(ostr.str().c_str());
    }

    const bool threaded = true;

    openvdb::util::NullInterrupter boss;

    // Check mask input
    const auto maskGeo = getInputCollection("mask VDB (optional)");
    openvdb::BoolGrid::Ptr maskGrid;

    if (maskGeo) {
        primitive_iterator maskIt(maskGeo, VDBVolume::id);

        if (maskIt.get()) {
			auto vdb = static_cast<VDBVolume *>(maskIt.get());

            MaskOp op;
            process_grid_real(vdb->getGrid(), vdb->storage(), op);
            maskGrid = op.mMaskGrid;
        }

		if (!maskGrid) {
			this->add_warning("Mask VDB not found.");
		}
    }

    // For each VDB primitive (with a non-null grid pointer) in the given group...
    std::string operationName;
    for (auto prim : primitive_iterator(m_collection, VDBVolume::id)) {
		if (boss.wasInterrupted()) {
			throw std::runtime_error("was interrupted");
		}

		auto vdb = static_cast<VDBVolume *>(prim);

        openvdb::GridBase::Ptr outGrid;
        bool ok = true;

        switch (whichOp) {
            case OP_GRADIENT: // gradient of scalar field
            {
                ToolOp<openvdb::tools::Gradient> op(threaded, boss, maskGrid.get());
                ok = process_grid_real(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_gradient";
                break;
            }
            case OP_CURVATURE: // mean curvature of scalar field
            {
                ToolOp<openvdb::tools::MeanCurvature> op(threaded, boss, maskGrid.get());
                ok = process_grid_real(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_curvature";
                break;
            }
            case OP_LAPLACIAN: // Laplacian of scalar field
            {
                ToolOp<openvdb::tools::Laplacian> op(threaded, boss, maskGrid.get());
                ok = process_grid_real(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_laplacian";
                break;
            }
            case OP_CPT: // closest point transform of scalar level set
            {
                ToolOp<openvdb::tools::Cpt> op(threaded, boss, maskGrid.get());
                ok = process_grid_real(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_cpt";
                break;
            }
            case OP_DIVERGENCE: // divergence of vector field
            {
                ToolOp<openvdb::tools::Divergence> op(threaded, boss, maskGrid.get());
                ok = process_grid_vector(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_divergence";
                break;
            }
            case OP_CURL: // curl (rotation) of vector field
            {
                ToolOp<openvdb::tools::Curl> op(threaded, boss, maskGrid.get());
                ok = process_grid_vector(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_curl";
                break;
            }
            case OP_MAGNITUDE: // magnitude of vector field
            {
                ToolOp<openvdb::tools::Magnitude> op(threaded, boss, maskGrid.get());
                ok = process_grid_vector(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_magnitude";
                break;
            }
            case OP_NORMALIZE: // normalize vector field
            {
                ToolOp<openvdb::tools::Normalize> op(threaded, boss, maskGrid.get());
                ok = process_grid_vector(vdb->getGrid(), vdb->storage(), op);

				if (ok) {
					outGrid = op.mOutGrid;
				}

                operationName = "_normalize";
                break;
            }
        }

        if (!ok) {
            auto inGridName = vdb->name();

            std::ostringstream ss;
            ss << "Can't compute " << operator_name[whichOp] << " from grid";

			if (!inGridName.empty()) {
				ss << " " << inGridName;
			}

            ss << " of type " << vdb->getGrid().valueType();

            this->add_warning(ss.str().c_str());
        }

        // Rename grid
        std::string gridName = vdb->getGrid().getName();
        const int renaming = eval_int("outputName");

        if (renaming == 1) {
            if (operationName.size() > 0)
				gridName += operationName;
        }
		else if (renaming == 2) {
            auto customName = eval_string("customName");

			if (customName.length() > 0) {
				gridName = customName;
			}
        }

		outGrid->setName(gridName);
		vdb->setGrid(outGrid);
    }
}

/* ************************************************************************** */

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBAnalysis);
}

}
