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

#include <openvdb/tools/Composite.h>
#include <openvdb/tools/GridTransformer.h> /* for resampleToMatch() */
#include <openvdb/tools/LevelSetRebuild.h> /* for levelSetRebuild() */
#include <openvdb/tools/Morphology.h> /* for deactivate() */
#include <openvdb/tools/SignedFloodFill.h>
#include <openvdb/tools/ChangeBackground.h>
#include <openvdb/tools/Prune.h>
#include <openvdb/util/NullInterrupter.h>

/* ************************************************************************** */

static constexpr auto NODE_NAME = "OpenVDB Combine";

class NodeOpenVDBCombine : public VDBNode {
public:
	NodeOpenVDBCombine();

	bool update_properties() override;

	void process() override;

	openvdb::GridBase::Ptr combineGrids(const int op,
	                                    openvdb::GridBase::ConstPtr aGrid,
	                                    openvdb::GridBase::Ptr bGrid,
	                                    const std::string &aGridName,
	                                    const std::string &bGridName,
	                                    const int resample);
};

/* ************************************************************************** */

enum {
	OP_COPY_A = 0,        /* A */
	OP_COPY_B,            /* B */
	OP_INVERT,            /* 1 - A */
	OP_ADD,               /* A + B */
	OP_SUBTRACT,          /* A - B */
	OP_MULTIPLY,          /* A * B */
	OP_DIVIDE,            /* A / B */
	OP_MAXIMUM,           /* max(A, B) */
	OP_MINIMUM,           /* min(A, B) */
	OP_BLEND1,            /* (1 - A) * B */
	OP_BLEND2,            /* A + (1 - A) * B */
	OP_UNION,             /* CSG A u B */
	OP_INTERSECTION,      /* CSG A n B */
	OP_DIFFERENCE,        /* CSG A / B */
	OP_REPLACE,           /* replace A with B */
	OP_TOPO_UNION,        /* A u active(B) */
	OP_TOPO_INTERSECTION, /* A n active(B) */
	OP_TOPO_DIFFERENCE    /* A / active(B) */
};

enum {
	RESAMPLE_OFF,    /* don't auto-resample grids */
	RESAMPLE_B,      /* resample B to match A */
	RESAMPLE_A,      /* resample A to match B */
	RESAMPLE_HI_RES, /* resample higher-res grid to match lower-res */
	RESAMPLE_LO_RES  /* resample lower-res grid to match higher-res */
};

enum {
	NEAREST = 0,
	LINEAR,
	QUADRATIC,
};

struct CombineOp;

static auto need_grid_a(int operation)
{
	return (operation != OP_COPY_B);
}

static auto need_grid_b(int operation)
{
	return (operation != OP_COPY_A && operation != OP_INVERT);
}

static auto need_level_sets(int operation)
{
	return (operation == OP_UNION || operation == OP_INTERSECTION || operation == OP_DIFFERENCE);
}

/* ************************************************************************** */

namespace {

/* Functor to compute scale * grid + offset, for scalars scale and offset */
template<typename GridT>
struct MulAdd {
	typedef typename GridT::ValueType ValueT;
	typedef typename GridT::Ptr GridPtrT;

	float scale, offset;

	explicit MulAdd(float s, float t = 0.0)
	    : scale(s)
	    , offset(t)
	{}

	void operator()(const ValueT &a, const ValueT&, ValueT &out) const
	{
		out = static_cast<ValueT>(a * scale + offset);
	}

	//* @return true if the scale is 1 and the offset is 0 */
	bool isIdentity() const
	{
		return (openvdb::math::isApproxEqual(scale, 1.f, 1.0e-6f)
		        && openvdb::math::isApproxEqual(offset, 0.f, 1.0e-6f));
	}

	//* Compute dest = src * scale + offset */
	void process(const GridT &src, GridPtrT &dest) const
	{
		if (isIdentity()) {
			dest = src.deepCopy();
			return;
		}

		if (!dest) {
			dest = GridT::create(src); /* same transform, new tree */
		}

		ValueT bg;
		(*this)(src.background(), ValueT(), bg);

		openvdb::tools::changeBackground(dest->tree(), bg);
		dest->tree().combine2(src.tree(), src.tree(), *this, /*prune=*/false);
	}
};

/* ************************************************************************** */

/* Functor to compute (1 - A) * B for grids A and B */
template<typename ValueT>
struct Blend1 {
	float aMult, bMult;
	const ValueT ONE;
	explicit Blend1(float a = 1.0, float b = 1.0)
	    : aMult(a)
	    , bMult(b)
	    , ONE(openvdb::zeroVal<ValueT>() + 1)
	{}

	void operator()(const ValueT &a, const ValueT &b, ValueT &out) const
	{
		out = ValueT((ONE - aMult * a) * bMult * b);
	}
};

/* ************************************************************************** */

//* Functor to compute A + (1 - A) * B for grids A and B */
template<typename ValueT>
struct Blend2 {
	float aMult, bMult;
	const ValueT ONE;

	explicit Blend2(float a = 1.0, float b = 1.0)
	    : aMult(a)
	    , bMult(b)
	    , ONE(openvdb::zeroVal<ValueT>() + 1)
	{}

	void operator()(const ValueT &a, const ValueT &b, ValueT &out) const
	{
		out = ValueT(a*aMult);
		out = out + ValueT((ONE - out) * bMult*b);
	}
};

/* ************************************************************************** */

/* Helper class to compare both scalar and vector values */
template<typename ValueT>
struct ApproxEq {
	const ValueT &a, &b;
	ApproxEq(const ValueT &_a, const ValueT &_b)
	    : a(_a)
	    , b(_b)
	{}

	operator bool() const
	{
		return openvdb::math::isRelOrApproxEqual(
		            a, b, /*rel*/ValueT(1e-6f), /*abs*/ValueT(1e-8f));
	}
};

/* Specialization for Vec2 */
template<typename T>
struct ApproxEq<openvdb::math::Vec2<T> > {
	typedef openvdb::math::Vec2<T> VecT;
	typedef typename VecT::value_type ValueT;

	const VecT &a, &b;

	ApproxEq(const VecT &_a, const VecT &_b)
	    : a(_a)
	    , b(_b)
	{}

	operator bool() const
	{
		return a.eq(b, /*abs=*/ValueT(1e-8f));
	}
};

/* Specialization for Vec3 */
template<typename T>
struct ApproxEq<openvdb::math::Vec3<T> > {
	typedef openvdb::math::Vec3<T> VecT;
	typedef typename VecT::value_type ValueT;

	const VecT &a, &b;

	ApproxEq(const VecT &_a, const VecT &_b)
	    : a(_a)
	    , b(_b)
	{}

	operator bool() const
	{
		return a.eq(b, /*abs=*/ValueT(1e-8f));
	}
};

/* Specialization for Vec4 */
template<typename T>
struct ApproxEq<openvdb::math::Vec4<T> > {
	typedef openvdb::math::Vec4<T> VecT;
	typedef typename VecT::value_type ValueT;

	const VecT &a, &b;

	ApproxEq(const VecT &_a, const VecT &_b)
	    : a(_a)
	    , b(_b)
	{}

	operator bool() const
	{
		return a.eq(b, /*abs=*/ValueT(1e-8f));
	}
};

template<typename T>
inline bool isFinite(const T &val)
{
	return std::isfinite(val);
}

inline bool isFinite(bool)
{
	return true;
}

} /* unnamed namespace */

/* ************************************************************************** */

template<typename AGridT>
struct DispatchOp {
	CombineOp *combineOp;

	DispatchOp(CombineOp &op)
	    : combineOp(&op)
	{}

	template<typename BGridT>
	void operator()(typename BGridT::ConstPtr);
};

/* ************************************************************************** */

/* Helper class for use with UTvdbProcessTypedGrid() */
struct CombineOp {
	NodeOpenVDBCombine *self = nullptr;
	int op;
	int resample;
	std::string aGridName, bGridName;
	openvdb::GridBase::ConstPtr aBaseGrid, bBaseGrid;
	openvdb::GridBase::Ptr outGrid;
	openvdb::util::NullInterrupter interupter;

	CombineOp() = default;

	/* Functor for use with UTvdbProcessTypedGridScalar() to return */
	/* a scalar grid's background value as a floating-point quantity */
	struct BackgroundOp {
		double value = 0.0;

		BackgroundOp() = default;

		template<typename GridT>
		void operator()(const GridT &grid)
		{
			value = static_cast<double>(grid.background());
		}
	};

	static double getScalarBackgroundValue(const openvdb::GridBase &baseGrid)
	{
		BackgroundOp bgOp;
		process_grid_real(baseGrid, get_grid_storage(baseGrid), bgOp);
		return bgOp.value;
	}

	template<typename GridT>
	typename GridT::Ptr resampleToMatch(const GridT &src, const openvdb::GridBase &ref, int order)
	{
		typedef typename GridT::ValueType ValueT;

		const auto ZERO = openvdb::zeroVal<ValueT>();
		const auto &refXform = ref.constTransform();

		typename GridT::Ptr dest;

		if (src.getGridClass() == openvdb::GRID_LEVEL_SET) {
			/* For level set grids, use the level set rebuild tool to both resample the */
			/* source grid to match the reference grid and to rebuild the resulting level set. */
			const ValueT halfWidth = ((ref.getGridClass() == openvdb::GRID_LEVEL_SET)
			                          ? ValueT(ZERO + this->getScalarBackgroundValue(ref) * (1.0 / ref.voxelSize()[0]))
			                         : ValueT(src.background() * (1.0 / src.voxelSize()[0])));

			if (!isFinite(halfWidth)) {
				std::stringstream msg;
				msg << "Resample to match: Illegal narrow band width = " << halfWidth
				    << ", caused by grid '" << src.getName() << "' with background "
				    << this->getScalarBackgroundValue(ref);
				throw std::invalid_argument(msg.str());
			}

			try {
				dest = openvdb::tools::doLevelSetRebuild(src, /*iso=*/ZERO,
				                                         /*exWidth=*/halfWidth, /*inWidth=*/halfWidth, &refXform, &interupter);
			}
			catch (openvdb::TypeError&) {
				std::stringstream ss;
				ss << "skipped rebuild of level set grid " + src.getName() + " of type " + src.type();
				self->add_warning(ss.str());
				dest.reset();
			}
		}

		if (!dest && src.constTransform() != refXform) {
			/* For non-level set grids or if level set rebuild failed due to an unsupported */
			/* grid type, use the grid transformer tool to resample the source grid to match */
			/* the reference grid. */
			dest = src.copy();
			dest->setTransform(refXform.copy());

			using namespace openvdb;

			switch (order) {
				case 0:
					tools::resampleToMatch<tools::PointSampler>(src, *dest);
					break;
				case 1:
					tools::resampleToMatch<tools::BoxSampler>(src, *dest);
					break;
				case 2:
					tools::resampleToMatch<tools::QuadraticSampler>(src, *dest);
					break;
			}
		}

		return dest;
	}

	/* If necessary, resample one grid so that its index space registers */
	/* with the other grid's. */
	/* Note that one of the grid pointers might change as a result. */
	template<typename AGridT, typename BGridT>
	void resampleGrids(const AGridT* &aGrid, const BGridT* &bGrid)
	{
		if (!aGrid || !bGrid) {
			return;
		}

		const auto needA = need_grid_a(op);
		const auto needB = need_grid_b(op);
		const auto needBoth = needA && needB;
		const auto samplingOrder = self->eval_int("Interpolation");

		/* One of RESAMPLE_A, RESAMPLE_B or RESAMPLE_OFF, specifying whether */
		/* grid A, grid B or neither grid was resampled */
		auto resampleWhich = static_cast<int>(RESAMPLE_OFF);

		/* Determine which of the two grids should be resampled. */
		if (resample == RESAMPLE_HI_RES || resample == RESAMPLE_LO_RES) {
			const auto aVoxSize = aGrid->voxelSize();
			const auto bVoxSize = bGrid->voxelSize();
			const auto aVoxVol = aVoxSize[0] * aVoxSize[1] * aVoxSize[2];
			const auto bVoxVol = bVoxSize[0] * bVoxSize[1] * bVoxSize[2];

			resampleWhich = ((aVoxVol > bVoxVol && resample == RESAMPLE_LO_RES)
			                 || (aVoxVol < bVoxVol && resample == RESAMPLE_HI_RES))
			                ? RESAMPLE_A : RESAMPLE_B;
		}
		else {
			resampleWhich = resample;
		}

		if (aGrid->constTransform() != bGrid->constTransform()) {
			/* If the A and B grid transforms don't match, one of the grids */
			/* should be resampled into the other's index space. */
			if (resample == RESAMPLE_OFF) {
				if (needBoth) {
					/* Resampling is disabled.  Just log a warning. */
					std::stringstream ss;
					ss << aGridName << " and " << bGridName << " transforms don't match\n";
					self->add_warning(ss.str());
				}
			}
			else {
				if (needA && resampleWhich == RESAMPLE_A) {
					/* Resample grid A into grid B's index space. */
					aBaseGrid = this->resampleToMatch(*aGrid, *bGrid, samplingOrder);
					aGrid = static_cast<const AGridT*>(aBaseGrid.get());
				}
				else if (needB && resampleWhich == RESAMPLE_B) {
					/* Resample grid B into grid A's index space. */
					bBaseGrid = this->resampleToMatch(*bGrid, *aGrid, samplingOrder);
					bGrid = static_cast<const BGridT*>(bBaseGrid.get());
				}
			}
		}

		if (aGrid->getGridClass() == openvdb::GRID_LEVEL_SET &&
		    bGrid->getGridClass() == openvdb::GRID_LEVEL_SET)
		{
			/* If both grids are level sets, ensure that their background values match. */
			/* (If one of the grids was resampled, then the background values should */
			/* already match.) */
			const double
			        a = this->getScalarBackgroundValue(*aGrid),
			        b = this->getScalarBackgroundValue(*bGrid);
			if (!ApproxEq<double>(a, b)) {
				if (resample == RESAMPLE_OFF) {
					if (needBoth) {
						/* Resampling/rebuilding is disabled.  Just log a warning. */
						std::stringstream ss;
						ss << aGridName << " and " << bGridName
						   << " background values don't match ("
						   << std::setprecision(3) << a << " vs. " << b << ");\n"
						   << "                 the output grid will not be a valid level set\n";
						self->add_warning(ss.str());
					}
				}
				else {
					/* One of the two grids needs a level set rebuild. */
					if (needA && resampleWhich == RESAMPLE_A) {
						/* Rebuild A to match B's background value. */
						aBaseGrid = this->resampleToMatch(*aGrid, *bGrid, samplingOrder);
						aGrid = static_cast<const AGridT*>(aBaseGrid.get());
					}
					else if (needB && resampleWhich == RESAMPLE_B) {
						/* Rebuild B to match A's background value. */
						bBaseGrid = this->resampleToMatch(*bGrid, *aGrid, samplingOrder);
						bGrid = static_cast<const BGridT*>(bBaseGrid.get());
					}
				}
			}
		}
	}

	void checkVectorTypes(const openvdb::GridBase* aGrid, const openvdb::GridBase* bGrid)
	{
		if (!aGrid || !bGrid || !need_grid_a(op) || !need_grid_b(op)) {
			return;
		}

		switch (op) {
			case OP_TOPO_UNION:
			case OP_TOPO_INTERSECTION:
			case OP_TOPO_DIFFERENCE:
				/* No need to warn about different vector types for topology-only operations. */
				break;

			default:
			{
				const auto aVecType = aGrid->getVectorType();
				const auto bVecType = bGrid->getVectorType();

				if (aVecType != bVecType) {
					std::stringstream ss;
					ss << aGridName << " and " << bGridName
					   << " have different vector types\n"
					   << "                 (" << openvdb::GridBase::vecTypeToString(aVecType)
					   << " vs. " << openvdb::GridBase::vecTypeToString(bVecType) << ")";
					self->add_warning(ss.str());
				}

				break;
			}
		}
	}

	/* Combine two grids of the same type. */
	template<typename GridT>
	void combineSameType()
	{
		typedef typename GridT::ValueType ValueT;

		const auto needA = need_grid_a(op);
		const auto needB = need_grid_b(op);
		const auto aMult = self->eval_float("A Multiplier");
		const auto bMult = self->eval_float("B Multiplier");

		const GridT *aGrid = NULL, *bGrid = NULL;

		if (aBaseGrid) {
			aGrid = VDB_grid_cast<GridT>(aBaseGrid).get();
		}

		if (bBaseGrid) {
			bGrid = VDB_grid_cast<GridT>(bBaseGrid).get();
		}

		if (needA && !aGrid) {
			throw std::runtime_error("missing A grid");
		}

		if (needB && !bGrid) {
			throw std::runtime_error("missing B grid");
		}

		/* Warn if combining vector grids with different vector types. */
		if (needA && needB && openvdb::VecTraits<ValueT>::IsVec) {
			this->checkVectorTypes(aGrid, bGrid);
		}

		/* If necessary, resample one grid so that its index space */
		/* registers with the other grid's. */
		if (aGrid && bGrid) {
			this->resampleGrids(aGrid, bGrid);
		}

		const auto ZERO = openvdb::zeroVal<ValueT>();

		/* A temporary grid is needed for binary operations, because they */
		/* cannibalize the B grid. */
		typename GridT::Ptr resultGrid, tempGrid;

		switch (op) {
			case OP_COPY_A:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				break;

			case OP_COPY_B:
				MulAdd<GridT>(bMult).process(*bGrid, resultGrid);
				break;

			case OP_INVERT:
				MulAdd<GridT>(-aMult, 1.0).process(*aGrid, resultGrid);
				break;

			case OP_ADD:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::compSum(*resultGrid, *tempGrid);
				break;

			case OP_SUBTRACT:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(-bMult).process(*bGrid, tempGrid);
				openvdb::tools::compSum(*resultGrid, *tempGrid);
				break;

			case OP_MULTIPLY:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::compMul(*resultGrid, *tempGrid);
				break;

			case OP_DIVIDE:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::compDiv(*resultGrid, *tempGrid);
				break;

			case OP_MAXIMUM:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::compMax(*resultGrid, *tempGrid);
				break;

			case OP_MINIMUM:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::compMin(*resultGrid, *tempGrid);
				break;

			case OP_BLEND1: /* (1 - A) * B */
			{
				const Blend1<ValueT> comp(aMult, bMult);
				ValueT bg;
				comp(aGrid->background(), ZERO, bg);
				resultGrid = aGrid->copy();

				openvdb::tools::changeBackground(resultGrid->tree(), bg);
				resultGrid->tree().combine2(aGrid->tree(), bGrid->tree(), comp, /*prune=*/false);
				break;
			}
			case OP_BLEND2: /* A + (1 - A) * B */
			{
				const Blend2<ValueT> comp(aMult, bMult);
				ValueT bg;
				comp(aGrid->background(), ZERO, bg);
				resultGrid = aGrid->copy();
				openvdb::tools::changeBackground(resultGrid->tree(), bg);
				resultGrid->tree().combine2(aGrid->tree(), bGrid->tree(), comp, /*prune=*/false);
				break;
			}

			case OP_UNION:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::csgUnion(*resultGrid, *tempGrid);
				break;

			case OP_INTERSECTION:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::csgIntersection(*resultGrid, *tempGrid);
				break;

			case OP_DIFFERENCE:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::csgDifference(*resultGrid, *tempGrid);
				break;

			case OP_REPLACE:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				MulAdd<GridT>(bMult).process(*bGrid, tempGrid);
				openvdb::tools::compReplace(*resultGrid, *tempGrid);
				break;

			case OP_TOPO_UNION:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				/* Note: no need to scale the B grid for topology-only operations. */
				resultGrid->topologyUnion(*bGrid);
				break;

			case OP_TOPO_INTERSECTION:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				resultGrid->topologyIntersection(*bGrid);
				break;

			case OP_TOPO_DIFFERENCE:
				MulAdd<GridT>(aMult).process(*aGrid, resultGrid);
				resultGrid->topologyDifference(*bGrid);
				break;
		}

		outGrid = this->postprocess<GridT>(resultGrid);
	}

	/* Combine two grids of different types. */
	//* @todo Currently, only topology operations can be performed on grids of different types. */
	template<typename AGridT, typename BGridT>
	void combineDifferentTypes()
	{
		const auto needA = need_grid_a(op);
		const auto needB = need_grid_b(op);

		const AGridT* aGrid = NULL;
		const BGridT* bGrid = NULL;

		if (aBaseGrid) {
			aGrid = VDB_grid_cast<AGridT>(aBaseGrid).get();
		}

		if (bBaseGrid) {
			bGrid = VDB_grid_cast<BGridT>(bBaseGrid).get();
		}

		if (needA && !aGrid) {
			throw std::runtime_error("missing A grid");
		}

		if (needB && !bGrid) {
			throw std::runtime_error("missing B grid");
		}

		/* Warn if combining vector grids with different vector types. */
		if (needA && needB && openvdb::VecTraits<typename AGridT::ValueType>::IsVec
		    && openvdb::VecTraits<typename BGridT::ValueType>::IsVec)
		{
			this->checkVectorTypes(aGrid, bGrid);
		}

		/* If necessary, resample one grid so that its index space */
		/* registers with the other grid's. */
		if (aGrid && bGrid) {
			this->resampleGrids(aGrid, bGrid);
		}

		const auto aMult = self->eval_float("A Multiplier");

		typename AGridT::Ptr resultGrid;

		switch (op) {
			case OP_TOPO_UNION:
				MulAdd<AGridT>(aMult).process(*aGrid, resultGrid);
				/* Note: no need to scale the B grid for topology-only operations. */
				resultGrid->topologyUnion(*bGrid);
				break;

			case OP_TOPO_INTERSECTION:
				MulAdd<AGridT>(aMult).process(*aGrid, resultGrid);
				resultGrid->topologyIntersection(*bGrid);
				break;

			case OP_TOPO_DIFFERENCE:
				MulAdd<AGridT>(aMult).process(*aGrid, resultGrid);
				resultGrid->topologyDifference(*bGrid);
				break;

			default:
			{
				std::ostringstream ostr;
				ostr << "can't combine grid " << aGridName << " of type " << aGrid->type()
				     << "\n                 with grid " << bGridName
				     << " of type " << bGrid->type();
				throw std::runtime_error(ostr.str());
				break;
			}
		}

		outGrid = this->postprocess<AGridT>(resultGrid);
	}

	template<typename GridT>
	typename GridT::Ptr postprocess(typename GridT::Ptr resultGrid)
	{
		typedef typename GridT::ValueType ValueT;

		const auto ZERO = openvdb::zeroVal<ValueT>();
		const auto prune = self->eval_bool("Prune");
		const auto flood = self->eval_bool("Signed-Flood-Fill Output SDFs");
		const auto deactivate = self->eval_bool("Deactivate Background Voxels");

		if (deactivate) {
			const auto deactivationTolerance = self->eval_float("Deactivate Tolerance");

			/* Mark active output tiles and voxels as inactive if their */
			/* values match the output grid's background value. */
			/* Do this first to facilitate pruning. */
			openvdb::tools::deactivate(*resultGrid, resultGrid->background(),
			                           ValueT(ZERO + deactivationTolerance));
		}

		if (flood && resultGrid->getGridClass() == openvdb::GRID_LEVEL_SET) {
			openvdb::tools::signedFloodFill(resultGrid->tree());
		}

		if (prune) {
			const float tolerance = self->eval_float("Prune Tolerance");
			openvdb::tools::prune(resultGrid->tree(), ValueT(ZERO + tolerance));
		}

		return resultGrid;
	}

	template<typename AGridT>
	void operator()(typename AGridT::ConstPtr)
	{
		const auto needA = need_grid_a(op);
		const auto needB = need_grid_b(op);
		const auto needBoth = needA && needB;

		if (!needBoth || !aBaseGrid || !bBaseGrid || aBaseGrid->type() == bBaseGrid->type()) {
			this->combineSameType<AGridT>();
		}
		else {
			DispatchOp<AGridT> dispatcher(*this);

			/* Dispatch on the B grid's type. */
			auto success = process_typed_grid(bBaseGrid,
			                                  get_grid_storage(*bBaseGrid),
			                                  dispatcher);

			if (!success) {
				std::stringstream ss;
				ss << "grid " << bGridName << " has unsupported type " << bBaseGrid->type();
				self->add_warning(ss.str());
			}
		}
	}
};

/* ************************************************************************** */

NodeOpenVDBCombine::NodeOpenVDBCombine()
    : VDBNode(NODE_NAME)
{
	addInput("input a");
	addInput("input b (optional)");
	addOutput("output");

	add_prop("Flatten All B into A", property_type::prop_bool);
	set_prop_default_value_bool(false);

	add_prop("Combine A/B Pairs", property_type::prop_bool);
	set_prop_default_value_bool(true);
	set_prop_tooltip("If disabled, combine each grid in group A\n"
	                 "with the first grid in group B.  Otherwise,\n"
	                 "pair A and B grids in the order that they\n"
	                 "appear in their respective groups.");

	/* Menu of available operations */
	EnumProperty operation_enum;
	operation_enum.insert("Copy A", OP_COPY_A);
	operation_enum.insert("Copy B", OP_COPY_B);
	operation_enum.insert("Invert A", OP_INVERT);
	operation_enum.insert("Add", OP_ADD);
	operation_enum.insert("Subtract", OP_SUBTRACT);
	operation_enum.insert("Multiply", OP_MULTIPLY);
	operation_enum.insert("Divide", OP_DIVIDE);
	operation_enum.insert("Maximum", OP_MAXIMUM);
	operation_enum.insert("Minimum", OP_MINIMUM);
	operation_enum.insert("(1 - A) \xd7 B", OP_BLEND1);
	operation_enum.insert("A + (1 - A) \xd7 B", OP_BLEND2);
	operation_enum.insert("SDF Union", OP_UNION);
	operation_enum.insert("SDF Intersection", OP_INTERSECTION);
	operation_enum.insert("SDF Difference", OP_DIFFERENCE);
	operation_enum.insert("Replace A with Active B", OP_REPLACE);
	operation_enum.insert("Activity Union", OP_TOPO_UNION);
	operation_enum.insert("Activity Intersection", OP_TOPO_INTERSECTION);
	operation_enum.insert("Activity Difference", OP_TOPO_DIFFERENCE);

	add_prop("Operation", property_type::prop_enum);
	set_prop_enum_values(operation_enum);

	add_prop("A Multiplier", property_type::prop_float);
	set_prop_min_max(-10.0f, 10.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Multiply voxel values in the A grid by a scalar\n"
	                 "before combining the A grid with the B grid.");

	add_prop("B Multiplier", property_type::prop_float);
	set_prop_min_max(-10.0f, 10.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Multiply voxel values in the B grid by a scalar\n"
	                 "before combining the A grid with the B grid.");

	/* Menu of resampling options */
	EnumProperty resample_enum;
	resample_enum.insert("Off", RESAMPLE_OFF);
	resample_enum.insert("B to Match A", RESAMPLE_B);
	resample_enum.insert("A to Match B", RESAMPLE_A);
	resample_enum.insert("Higher-res to Match Lower-res", RESAMPLE_HI_RES);
	resample_enum.insert("Lower-res to Match Higher-res", RESAMPLE_LO_RES);

	add_prop("Resample", property_type::prop_enum);
	set_prop_enum_values(resample_enum);
	set_prop_tooltip("If the A and B grids have different transforms, one grid should\n"
	                 "be resampled to match the other before the two are combined.\n"
	                 "Also, level set grids should have matching background values.\n");

	/* Menu of resampling interpolation order options */
	EnumProperty interpolation_enum;
	interpolation_enum.insert("Nearest",   NEAREST);
	interpolation_enum.insert("Linear",    LINEAR);
	interpolation_enum.insert("Quadratic", QUADRATIC);

	add_prop("Interpolation", property_type::prop_enum);
	set_prop_enum_values(interpolation_enum);
	set_prop_tooltip("Specify the type of interpolation to be used when\n"
	                 "resampling one grid to match the other's transform.");

	add_prop("Deactivate Background Voxels", property_type::prop_bool);
	set_prop_default_value_bool(false);

	add_prop("Deactivate Tolerance", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(0.0f);
	set_prop_tooltip("Deactivate active output voxels whose values\n"
	                 "equal the output grid's background value.\n"
	                 "Voxel values are considered equal if they differ\n"
	                 "by less than the specified tolerance.");

	add_prop("Prune", property_type::prop_bool);
	set_prop_default_value_bool(false);

	add_prop("Prune Tolerance", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(0.0f);
	set_prop_tooltip("Collapse regions of constant value in output grids.\n"
	                 "Voxel values are considered equal if they differ\n"
	                 "by less than the specified tolerance.");

	add_prop("Signed-Flood-Fill Output SDFs", property_type::prop_bool);
	set_prop_default_value_bool(false);
	set_prop_tooltip("Reclassify inactive voxels of level set grids as either inside or outside.");
}

bool NodeOpenVDBCombine::update_properties()
{
	set_prop_visible("Interpolation", eval_enum("Resample") != RESAMPLE_OFF);
	set_prop_visible("Deactivate Tolerance", eval_bool("Deactivate Background Voxels"));
	set_prop_visible("Prune Tolerance", eval_bool("Prune"));
	set_prop_visible("Combine A/B Pairs", eval_bool("Flatten All B into A") == false);
	return true;
}

void NodeOpenVDBCombine::process()
{
	const auto pairs = eval_bool("Combine A/B Pairs");
	const auto flatten = eval_bool("Flatten All B into A");
	const auto op = eval_int("Operation");
	const auto needA = need_grid_a(op);
	const auto needB = need_grid_b(op);
	const auto resample = eval_int("Resample");

	std::vector<Primitive *> prims_to_delete;
	PrimitiveCollection created_prims(m_collection->factory());

	const auto a_collection = m_collection;
	const auto b_collection = getInputCollection("input b (optional)");

	/* Iterate over A and, optionally, B grids. */
	primitive_iterator aIt(a_collection);
	primitive_iterator bIt(b_collection);

	for ( ; (!needA || (aIt.get() != nullptr)) && (!needB || (bIt.get() != nullptr));
	      ++aIt, ((needB && pairs) ? ++bIt : bIt))
	{
		/* Note: even if needA is false, we still need to delete A grids. */
		auto aVdb = static_cast<VDBVolume *>(aIt.get());
		auto bVdb = static_cast<VDBVolume *>(bIt.get());

		openvdb::GridBase::Ptr aGrid;
		openvdb::GridBase::Ptr bGrid;

		if (aVdb) {
			aGrid = aVdb->getGridPtr();
		}

		if (bVdb) {
			bGrid = bVdb->getGridPtr();
		}

		/* For error reporting, get the names of the A and B grids. */
		auto aGridName = (aVdb != nullptr) ? aVdb->name() : "A";
		auto bGridName = (bVdb != nullptr) ? bVdb->name() : "B";

		/* Name the output grid after the A grid, except (see below) if
				 * the A grid is unused. */
		auto outGridName = aGridName;

		openvdb::GridBase::Ptr outGrid;

		while (true) {
			/* If the A grid is unused, name the output grid after the
			 * most recent B grid. */
			if (!needA) {
				outGridName = bGridName;
			}

			outGrid = combineGrids(op, aGrid, bGrid, aGridName, bGridName, resample);

			/* When not flattening, quit after one pass. */
			if (!flatten) {
				break;
			}

			/* See if we have any more B grids. */
			++bIt;
			if (!bIt.get()) {
				break;
			}

			bVdb = static_cast<VDBVolume *>(*bIt);
			bGrid = bVdb->getGridPtr();
			bGridName = (bVdb != nullptr) ? bVdb->name() : "B";

			aGrid = outGrid;
			if (!aGrid) {
				break;
			}
		}

		if (outGrid) {
			/* Add a new VDB primitive for the output grid to the output gdp. */
			build_vdb_prim(&created_prims, outGrid);

			/* Remove the A grid from the output gdp. */
			if (aVdb) {
				prims_to_delete.push_back(aVdb);
			}
		}

		if (!needA && !pairs) {
			break;
		}

		if (flatten) {
			break;
		}
	}

	/* In non-paired mode, there should be only one B grid. */
	if (!pairs && !flatten) {
		++bIt;
	}

	/* In flatten mode there should be a single A grid. */
	if (flatten) {
		++aIt;
	}

	const auto unusedA = (needA && (aIt.get() != nullptr));
	const auto unusedB = (needB && (bIt.get() != nullptr));

	if (unusedA || unusedB) {
		std::stringstream ss;
		ss << "some grids were not processed because there were more "
		   << (unusedA ? "A" : "B") << " grids than "
		   << (unusedA ? "B" : "A") << " grids\n";
		this->add_warning(ss.str());
	}

	/* Remove processed prims. */
	m_collection->destroy(prims_to_delete);

	/* Copy created prims to output. */
	m_collection->merge_collection(created_prims);
}

openvdb::GridBase::Ptr NodeOpenVDBCombine::combineGrids(const int op,
                                                        openvdb::GridBase::ConstPtr aGrid,
                                                        openvdb::GridBase::Ptr bGrid,
                                                        const std::string &aGridName,
                                                        const std::string &bGridName,
                                                        const int resample)
{
	openvdb::GridBase::Ptr outGrid;

	const auto needA = need_grid_a(op);
	const auto needB = need_grid_b(op);
	const auto needLS = need_level_sets(op);

	if (!needA && !needB) {
		throw std::runtime_error("Nothing to do.");
	}

	if (needA && !aGrid) {
		throw std::runtime_error("Missing A grid.");
	}

	if (needB && !bGrid) {
		throw std::runtime_error("Missing B grid.");
	}

	if (needLS &&
	    ((aGrid && aGrid->getGridClass() != openvdb::GRID_LEVEL_SET) ||
	     (bGrid && bGrid->getGridClass() != openvdb::GRID_LEVEL_SET)))
	{
		std::stringstream ss;
		ss << "expected level set grids for the operation,\n                 found "
		   << openvdb::GridBase::gridClassToString(aGrid->getGridClass()) << " (" << aGridName << ") and "
		   << openvdb::GridBase::gridClassToString(bGrid->getGridClass()) << " (" << bGridName
		   << ");\n                 the output grid will not be a valid level set\n";
		this->add_warning(ss.str());
	}

	if (needA && needB && aGrid->type() != bGrid->type()
	    && op != OP_TOPO_UNION && op != OP_TOPO_INTERSECTION && op != OP_TOPO_DIFFERENCE)
	{
		std::stringstream ss;
		ss << "Can't combine grid " << aGridName << " of type " << aGrid->type()
		   << "\n                 with grid " << bGridName << " of type " << bGrid->type();
		this->add_warning(ss.str());
		return outGrid;
	}

	CombineOp compOp;
	compOp.self = this;
	compOp.op = op;
	compOp.resample = resample;
	compOp.aBaseGrid = aGrid;
	compOp.bBaseGrid = bGrid;
	compOp.aGridName = aGridName;
	compOp.bGridName = bGridName;

#if 0
	auto success = process_grid_real(aGrid,
	                                 get_grid_storage(needA ? *aGrid : *bGrid),
	                                 compOp);
#else
	auto success = false;
#endif

	if (!success || !compOp.outGrid) {
		std::stringstream ss;

		if (aGrid->type() == bGrid->type()) {
			ss << "Grids " << aGridName << " and " << bGridName
			   << " have unsupported type " << aGrid->type();
		}
		else {
			ss << "Grid " << (needA ? aGridName : bGridName)
			   << " has unsupported type " << (needA ? aGrid->type() : bGrid->type());
		}

		this->add_warning(ss.str());
	}

	return compOp.outGrid;
}

/* ************************************************************************** */

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBCombine);
}

}
