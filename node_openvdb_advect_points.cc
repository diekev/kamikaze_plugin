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
#include <kamikaze/outils/parallélisme.h>

#include <openvdb/tools/PointAdvect.h>
#include <openvdb/util/NullInterrupter.h>

#include "util_openvdb_process.h"
#include "volumebase.h"

namespace {

// Add new items to the *end* of this list, and update NUM_PROPAGATION_TYPES.
enum PropagationType {
    PROPAGATION_TYPE_UNKNOWN = -1,
    PROPAGATION_TYPE_ADVECTION = 0,
    PROPAGATION_TYPE_PROJECTION,
    PROPAGATION_TYPE_CONSTRAINED_ADVECTION
};

enum { NUM_PROPAGATION_TYPES = PROPAGATION_TYPE_CONSTRAINED_ADVECTION + 1 };

// Add new items to the *end* of this list, and update NUM_INTEGRATION_TYPES.
enum IntegrationType {
    INTEGRATION_TYPE_UNKNOWN = -1,
    INTEGRATION_TYPE_FWD_EULER = 0,
    INTEGRATION_TYPE_RK_2ND,
    INTEGRATION_TYPE_RK_3RD,
    INTEGRATION_TYPE_RK_4TH
};

enum { NUM_INTEGRATION_TYPES = INTEGRATION_TYPE_RK_4TH + 1 };

struct AdvectionParms {
    AdvectionParms(PrimitiveCollection *outputGeo)
        : mOutputGeo(outputGeo)
    {}

    PrimitiveCollection *mOutputGeo = nullptr;
    PrimPoints* mPointGeo = nullptr;
    const VDBVolume *mVelPrim = nullptr;
    const VDBVolume *mCptPrim = nullptr;
    PropagationType mPropagationType = PROPAGATION_TYPE_ADVECTION;
    IntegrationType mIntegrationType = INTEGRATION_TYPE_FWD_EULER;
    double mTimeStep = 1.0;
    int mIterations = 1;
	int mSteps = 1;
    bool mStaggered = false;
	bool mStreamlines = false;
};

#if 0
/// @brief Creates a new line segment for each point in @c ptnGeo
/// @note The lines will only have one node.
void
createNewLines(GU_Detail& geo, const GA_PointGroup* group)
{
    GA_SplittableRange ptnRange(geo.getPointRange(group));
    GA_Offset start, end, pt;

    for (GA_PageIterator pIt = ptnRange.beginPages(); !pIt.atEnd(); ++pIt) {
        for (GA_Iterator it(pIt.begin()); it.blockAdvance(start, end); ) {
            for (GA_Offset i = start; i < end; ++i) {

                pt = geo.appendPointOffset();
                geo.setPos3(pt, geo.getPos3(i));

                GU_PrimPoly& prim = *GU_PrimPoly::build(&geo, 0, GU_POLY_OPEN, 0);
                prim.appendVertex(pt);
            }
        }
    }
}


/// @brief Append a new node to each line.
/// @note The numbers of lines and points have to match.
void
appendLineNodes(GU_Detail& geo, GA_Size firstline, const GU_Detail& ptnGeo)
{
    GA_SplittableRange ptnRange(ptnGeo.getPointRange());
    GA_Offset start, end, pt;

    GA_Size n = firstline, N = geo.getNumPrimitives();


    for (GA_PageIterator pIt = ptnRange.beginPages(); !pIt.atEnd(); ++pIt) {
        for (GA_Iterator it(pIt.begin()); it.blockAdvance(start, end); ) {
            for (GA_Offset i = start; i < end; ++i) {

                pt = geo.appendPointOffset();
                geo.setPos3(pt, ptnGeo.getPos3(i));

                GA_Offset offset = geo.primitiveOffset(n);
                GU_PrimPoly& prim = *static_cast<GU_PrimPoly*>(
                    geo.getPrimitiveList().get(offset));

                prim.appendVertex(pt);

                if (++n == N) break;
            }
            if (n == N) break;
        }
        if (n == N) break;
    }
}
#endif

// Threaded closest point projection
template<typename GridType>
class ProjectionOp {
    typedef openvdb::tools::ClosestPointProjector<GridType> ProjectorType;
    typedef typename GridType::ValueType VectorType;
    typedef typename VectorType::ValueType ElementType;

	ProjectorType m_projector;
    PointList &m_points;
    openvdb::util::NullInterrupter& m_boss;

public:

    ProjectionOp(const GridType& cptGrid,
	             int cptIterations,
	             PointList &points,
	             openvdb::util::NullInterrupter& boss)
        : m_projector(cptGrid, cptIterations)
        , m_points(points)
        , m_boss(boss)
    {}

    void operator()(const tbb::blocked_range<size_t> &range) const
    {
        glm::vec3 p;
        VectorType w;

		for (auto i = range.begin(); i < range.end(); ++i) {
			p = m_points[i];
			w[0] = ElementType(p[0]);
            w[1] = ElementType(p[1]);
            w[2] = ElementType(p[2]);

			m_projector.projectToConstraintSurface(w);

			p[0] = static_cast<glm::vec3::value_type>(w[0]);
            p[1] = static_cast<glm::vec3::value_type>(w[1]);
            p[2] = static_cast<glm::vec3::value_type>(w[2]);
			m_points[i] = p;
		}
    }
};

class Projection {
	AdvectionParms& mParms;
    openvdb::util::NullInterrupter& m_boss;

public:
    Projection(AdvectionParms& parms, openvdb::util::NullInterrupter& boss)
        : mParms(parms)
        , m_boss(boss)
    {}

    template<typename GridType>
    void operator()(const GridType& grid)
    {
		if (m_boss.wasInterrupted()) {
			return;
		}

		auto points = mParms.mPointGeo->points();
        ProjectionOp<GridType> op(grid, mParms.mIterations, *points, m_boss);
        parallel_for(tbb::blocked_range<size_t>(0, points->size()), op);
    }
};

// Threaded point advection
template<typename GridType, int IntegrationOrder, bool StaggeredVelocity, bool Constrained = false>
class AdvectionOp {
    typedef openvdb::tools::VelocityIntegrator<GridType, StaggeredVelocity> IntegrationType;
    typedef openvdb::tools::ClosestPointProjector<GridType> ProjectorType; // Used for constrained advection

    typedef typename GridType::ValueType VectorType;
    typedef typename VectorType::ValueType ElementType;

	const GridType& mVelocityGrid;
    const GridType* mCptGrid = nullptr;
    PointList &m_points;
    openvdb::util::NullInterrupter& m_boss;
    double mTimeStep;
    Attribute *mTrailLen = nullptr;
    const int mSteps;
	const int mCptIterations = 0;

public:

    AdvectionOp(const GridType& velocityGrid, PointList &geo, openvdb::util::NullInterrupter& boss,
	            double timeStep, Attribute *traillen, int steps)
        : mVelocityGrid(velocityGrid)
        , m_points(geo)
        , m_boss(boss)
        , mTimeStep(timeStep)
        , mTrailLen(traillen)
        , mSteps(steps)
    {}

    AdvectionOp(const GridType& velocityGrid, const GridType& cptGrid, PointList &geo,
	            openvdb::util::NullInterrupter& boss, double timeStep, int steps, int cptIterations)
        : mVelocityGrid(velocityGrid)
        , mCptGrid(&cptGrid)
        , m_points(geo)
        , m_boss(boss)
        , mTimeStep(timeStep)
        , mSteps(steps)
        , mCptIterations(cptIterations)
    {}

    void operator()(const tbb::blocked_range<size_t> &range) const
    {
        IntegrationType integrator(mVelocityGrid);

        // Constrained-advection compiled out if Constrained == false
        std::unique_ptr<ProjectorType> projector(nullptr);
        if (Constrained && mCptGrid != nullptr) {
            projector.reset(new ProjectorType(*mCptGrid, mCptIterations));
        }

		glm::vec3 p;
        VectorType w;

		for (auto i = range.begin(); i < range.end(); ++i) {
			p = m_points[i];
			w[0] = ElementType(p[0]);
            w[1] = ElementType(p[1]);
            w[2] = ElementType(p[2]);

			ElementType timestep = static_cast<ElementType>(mTimeStep);
            if (mTrailLen != nullptr) {
                timestep *= static_cast<ElementType>(mTrailLen->float_(i));
            }

			for (int n = 0; n < mSteps; ++n) {
                integrator.template rungeKutta<IntegrationOrder, VectorType>(timestep, w);

				if (Constrained) {
					projector->projectToConstraintSurface(w);
				}
            }

			p[0] = static_cast<glm::vec3::value_type>(w[0]);
            p[1] = static_cast<glm::vec3::value_type>(w[1]);
            p[2] = static_cast<glm::vec3::value_type>(w[2]);
			m_points[i] = p;
		}
    }
};

class Advection {
	AdvectionParms&    mParms;
    openvdb::util::NullInterrupter& m_boss;

public:
    Advection(AdvectionParms& parms, openvdb::util::NullInterrupter& boss)
        : mParms(parms)
        , m_boss(boss)
    {}

    template<typename GridType, int IntegrationOrder, bool StaggeredVelocity>
    void advection(const GridType& velocityGrid)
    {
		if (m_boss.wasInterrupted()) {
			return;
		}

        if (!mParms.mStreamlines) { // Advect points

			auto trail_attrib = mParms.mPointGeo->attribute("traillen", ATTR_TYPE_FLOAT);

            AdvectionOp<GridType, IntegrationOrder, StaggeredVelocity>
                op(velocityGrid, *mParms.mPointGeo->points(), m_boss, mParms.mTimeStep,
                    trail_attrib, mParms.mSteps);

            parallel_for(
                tbb::blocked_range<size_t>(0, mParms.mPointGeo->points()->size()), op);

        }
		else { // Advect points and generate streamlines.
#if 0
            GA_Index firstline = mParms.mOutputGeo->getNumPrimitives();

            GU_Detail geo;
            geo.mergePoints(*mParms.mPointGeo, mParms.mPointGroup);

            createNewLines(*mParms.mOutputGeo, mParms.mPointGroup);

            for (int n = 0; n < mParms.mSteps; ++n) {

				if (m_boss.wasInterrupted()) {
					return;
				}

				auto trail_attrib = mParms.mPointGeo->attribute("traillen", ATTR_TYPE_FLOAT);

                AdvectionOp<GridType, IntegrationOrder, StaggeredVelocity>
                    op(velocityGrid, geo, m_boss, mParms.mTimeStep, traillen_h, 1);

                UTparallelFor(GA_SplittableRange(geo.getPointRange()), op);

                appendLineNodes(*mParms.mOutputGeo, firstline, geo);
			}
#endif
        }
    }

    template<typename GridType, int IntegrationOrder, bool StaggeredVelocity>
    void constrainedAdvection(const GridType& velocityGrid)
    {
        const GridType& cptGrid = static_cast<const GridType&>(mParms.mCptPrim->getGrid());
        typedef AdvectionOp<GridType, IntegrationOrder, StaggeredVelocity, /*Constrained*/true>
            AdvectionOp;

		if (m_boss.wasInterrupted()) {
			return;
		}

        if (!mParms.mStreamlines) { // Advect points
            AdvectionOp op(velocityGrid, cptGrid, *mParms.mPointGeo->points(), m_boss,
                mParms.mTimeStep, mParms.mSteps, mParms.mIterations);

			parallel_for(
                tbb::blocked_range<size_t>(0, mParms.mPointGeo->points()->size()), op);
        }
		else { // Advect points and generate streamlines.
#if 0
            GA_Index firstline = mParms.mOutputGeo->getNumPrimitives();

            GU_Detail geo;
            geo.mergePoints(*mParms.mPointGeo, mParms.mPointGroup);

            createNewLines(*mParms.mOutputGeo, mParms.mPointGroup);

            for (int n = 0; n < mParms.mSteps; ++n) {

				if (m_boss.wasInterrupted()) {
					return;
				}

                AdvectionOp op(velocityGrid, cptGrid, geo, m_boss,
                    mParms.mTimeStep, 1, mParms.mIterations);

                UTparallelFor(GA_SplittableRange(geo.getPointRange()), op);

                appendLineNodes(*mParms.mOutputGeo, firstline, geo);
            }
#endif
        }
    }

    // Resolves velocity representation and advection type
    template<typename GridType, int IntegrationOrder>
    void resolveAdvection(const GridType& velocityGrid)
    {
		if (m_boss.wasInterrupted()) {
			return;
		}

        if (mParms.mPropagationType == PROPAGATION_TYPE_ADVECTION) {
			if (!mParms.mStaggered) {
				advection<GridType, IntegrationOrder, false>(velocityGrid);
			}
			else {
				advection<GridType, IntegrationOrder, true>(velocityGrid);
			}
        }
		else if (mParms.mCptPrim != nullptr) { // constrained
            if (!mParms.mStaggered) {
                constrainedAdvection<GridType, IntegrationOrder, false>(velocityGrid);
            }
			else {
                constrainedAdvection<GridType, IntegrationOrder, true>(velocityGrid);
            }
        }
    }

    template<typename GridType>
    void operator()(const GridType& velocityGrid)
    {
        if (m_boss.wasInterrupted()) return;

        // Resolve integration order
        switch (mParms.mIntegrationType) {
            case INTEGRATION_TYPE_FWD_EULER: resolveAdvection<GridType, 1>(velocityGrid); break;
            case INTEGRATION_TYPE_RK_2ND:    resolveAdvection<GridType, 2>(velocityGrid); break;
            case INTEGRATION_TYPE_RK_3RD:    resolveAdvection<GridType, 3>(velocityGrid); break;
            case INTEGRATION_TYPE_RK_4TH:    resolveAdvection<GridType, 4>(velocityGrid); break;
            case INTEGRATION_TYPE_UNKNOWN: break;
        }
    }
};

}  /* namespace */

/* ************************************************************************** */

static constexpr auto NOM_OPERATEUR = "OpenVDB Advect Points";
static constexpr auto AIDE_OPERATEUR = "";

class NodeOpenVDBAdvectPoints : public OperateurOpenVDB {
public:
	NodeOpenVDBAdvectPoints(Noeud *noeud, const Context &contexte);
	~NodeOpenVDBAdvectPoints() = default;

	const char *nom_entree(size_t index) override
	{
		switch (index) {
			default:
			case 0:
				return "input";
			case 1:
				return "velocity VDB (optional)";
			case 2:
				return "closest point VDB (optional)";
		}
	}

	const char *nom_sortie(size_t /*index*/) override { return "output"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;
	bool update_properties() override;
	bool evalAdvectionParms(AdvectionParms &parms, const Context &contexte, double temps);
};

NodeOpenVDBAdvectPoints::NodeOpenVDBAdvectPoints(Noeud *noeud, const Context &contexte)
	: OperateurOpenVDB(noeud, contexte)
{
	entrees(3);
	sorties(1);

    /* Propagation scheme. */
    {
		EnumProperty items;
		items.insert("Advection", PROPAGATION_TYPE_ADVECTION);
		items.insert("Projection", PROPAGATION_TYPE_PROJECTION);
		items.insert("Constrained Advection", PROPAGATION_TYPE_CONSTRAINED_ADVECTION);
		items.insert("Unknown", PROPAGATION_TYPE_UNKNOWN);

		add_prop("propagation", "Operation", property_type::prop_enum);
		set_prop_enum_values(items);
		set_prop_default_value_int(0);
		set_prop_tooltip("Advection: Move the point along the velocity field.\n"
		                 "Projection: Move point to the nearest surface point.\n"
		                 "Projected advection: Advect, then project to the nearest surface point.");
    }

    /* Integration scheme. */
    {
		EnumProperty items;
		items.insert("Forward Euler", INTEGRATION_TYPE_FWD_EULER);
		items.insert("Second-Order Runge-Kutta", INTEGRATION_TYPE_RK_2ND);
		items.insert("Third-Order Runge-Kutta", INTEGRATION_TYPE_RK_3RD);
		items.insert("Fourth-Order Runge-Kutta", INTEGRATION_TYPE_RK_4TH);
		items.insert("Unknown", INTEGRATION_TYPE_UNKNOWN);

		add_prop("integration", "Integration", property_type::prop_enum);
		set_prop_enum_values(items);
		set_prop_default_value_int(0);
		set_prop_tooltip("Lower order means faster performance, "
		                 "but the points will not follow the velocity field closely.");
    }

    /* Closest point iterations */
	add_prop("cptIterations", "Iterations", property_type::prop_int);
	set_prop_min_max(1, 10);
	set_prop_tooltip("The interpolation step when sampling nearest points introduces\n"
	                 "error so that the result of a single sample may not lie exactly\n"
		             "on the surface. Multiple iterations help minimize this error.");

    /* Time step */
	add_prop("timeStep", "Time Step", property_type::prop_float);
	set_prop_default_value_float(1.0f / 24.0f);
	set_prop_min_max(0.0f, 10.0f);

    /* Steps */
	add_prop("steps", "Substeps", property_type::prop_int);
	set_prop_default_value_int(1);
	set_prop_min_max(1, 10);
	set_prop_tooltip("Number of timesteps to take per frame.");

    /* Output streamlines */
	add_prop("outputStreamlines", "Output Streamlines", property_type::prop_bool);
	set_prop_min_max(1, 10);
	set_prop_tooltip("Output the particle path as line segments.");
}

bool NodeOpenVDBAdvectPoints::update_properties()
{
    const auto propagation = static_cast<PropagationType>(eval_int("propagation"));

    set_prop_visible("cptIterations", propagation != PROPAGATION_TYPE_ADVECTION);
    set_prop_visible("integration", propagation != PROPAGATION_TYPE_PROJECTION);
    set_prop_visible("timeStep", propagation != PROPAGATION_TYPE_PROJECTION);
    set_prop_visible("steps", propagation != PROPAGATION_TYPE_PROJECTION);
    set_prop_visible("outputStreamlines", propagation != PROPAGATION_TYPE_PROJECTION);

    return true;
}

void NodeOpenVDBAdvectPoints::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	AdvectionParms parms(m_collection);
	if (!evalAdvectionParms(parms, contexte, temps)) {
		return;
	}

    openvdb::util::NullInterrupter boss;

    switch (parms.mPropagationType) {
        case PROPAGATION_TYPE_ADVECTION:
        case PROPAGATION_TYPE_CONSTRAINED_ADVECTION:
        {
            Advection advection(parms, boss);
            process_grid_vector(parms.mVelPrim->getGrid(), parms.mVelPrim->storage(), advection);
            break;
        }
        case PROPAGATION_TYPE_PROJECTION:
        {
            Projection projection(parms, boss);
            process_grid_vector(parms.mCptPrim->getGrid(), parms.mCptPrim->storage(), projection);
            break;
        }
        case PROPAGATION_TYPE_UNKNOWN:
		{
			break;
		}
    }

	if (boss.wasInterrupted()) {
		this->ajoute_avertissement("Processing was interrupted!");
	}

    boss.end();
}

bool NodeOpenVDBAdvectPoints::evalAdvectionParms(AdvectionParms& parms, const Context &contexte, double temps)
{
	primitive_iterator pit(m_collection, PrimPoints::id);
    parms.mPointGeo = static_cast<PrimPoints *>(*pit);

    if (!parms.mPointGeo) {
		this->ajoute_avertissement("Missing point input");
        return false;
    }

    parms.mPropagationType = static_cast<PropagationType>(eval_int("propagation"));

    if (parms.mPropagationType == PROPAGATION_TYPE_UNKNOWN) {
		this->ajoute_avertissement("Unknown propargation scheme");
        return false;
    }

    if (parms.mPropagationType == PROPAGATION_TYPE_ADVECTION ||
        parms.mPropagationType == PROPAGATION_TYPE_CONSTRAINED_ADVECTION) {

		const PrimitiveCollection *velGeo = entree(1)->requiers_collection(nullptr, contexte, temps);

        if (!velGeo) {
			this->ajoute_avertissement("Missing velocity grid input");
            return false;
        }

		primitive_iterator it(velGeo, VDBVolume::id);
        parms.mVelPrim = static_cast<VDBVolume *>(*it);

        if (!parms.mVelPrim) {
			this->ajoute_avertissement("Missing velocity grid");
            return false;
        }

        if (parms.mVelPrim->storage() != GRID_STORAGE_VEC3S) {
			this->ajoute_avertissement("Expected velocity grid to be of type Vec3f");
            return false;
        }

        // Check if the velocity grid uses a staggered representation.
        parms.mStaggered =
            parms.mVelPrim->getGrid().getGridClass() == openvdb::GRID_STAGGERED;

        parms.mTimeStep = eval_float("timeStep");
        parms.mSteps    = eval_int("steps");

        // The underlying code will accumulate, so to make it substeps
        // we need to divide out.
        parms.mTimeStep /= static_cast<float>(parms.mSteps);
        parms.mStreamlines  = eval_bool("outputStreamlines");

        parms.mIntegrationType = static_cast<IntegrationType>(eval_enum("integration"));

        if (parms.mIntegrationType == INTEGRATION_TYPE_UNKNOWN) {
			this->ajoute_avertissement("Unknown integration scheme");
            return false;
        }
    }

    if (parms.mPropagationType == PROPAGATION_TYPE_PROJECTION ||
        parms.mPropagationType == PROPAGATION_TYPE_CONSTRAINED_ADVECTION)
	{
		const PrimitiveCollection *cptGeo = entree(2)->requiers_collection(nullptr, contexte, temps);;

        if (!cptGeo) {
			this->ajoute_avertissement("Missing closest point grid input");
            return false;
        }

		primitive_iterator it(cptGeo, VDBVolume::id);
        parms.mCptPrim = static_cast<VDBVolume *>(*it);

        if (!parms.mCptPrim) {
			this->ajoute_avertissement("Missing closest point grid");
            return false;
        }

        if (parms.mCptPrim->storage() != GRID_STORAGE_VEC3S) {
			this->ajoute_avertissement("Expected closest point grid to be of type Vec3f");
            return false;
        }

        parms.mIterations = eval_int("cptIterations");
    }

    return true;
}

/* ************************************************************************** */

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeOpenVDBAdvectPoints>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
