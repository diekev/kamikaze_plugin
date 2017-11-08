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

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/tools/PointsToMask.h>
#include <openvdb/tools/TopologyToLevelSet.h>

#include "volumebase.h"

/* ************************************************************************** */

/* This wrapper class is required by openvdb::tools::ParticlesToLeveSet */
class ParticleList {
	PointList *m_points;
	Attribute *m_scale;
	Attribute *m_velocity;
	bool m_has_radius;
	bool m_has_velocity;
	float m_rad_mult;
	float m_vel_mult;

public:
    // Required by @c openvdb::tools::PointPartitioner
    typedef openvdb::Vec3R  PosType;

    ParticleList(PrimPoints *prim, float rad_mult = 1.0f, float vel_mult = 1.0f)
	    : m_points(prim->points())
	    , m_scale(prim->attribute("scale", ATTR_TYPE_FLOAT))
	    , m_velocity(prim->attribute("velocity", ATTR_TYPE_VEC3))
	    , m_has_radius(m_scale != nullptr)
	    , m_has_velocity(m_velocity != nullptr)
	    , m_rad_mult(rad_mult)
	    , m_vel_mult(vel_mult)
    {}

    /* Do the particles have non-constant radius */
    bool hasRadius() const
	{
		return m_has_radius;
	}

    /* Do the particles have velocity */
    bool hasVelocity() const
	{
		return m_has_velocity;
	}

    // Multiplier for the radius
    float &radiusMult()
	{
		return m_rad_mult;
	}

    const float &radiusMult() const
	{
		return m_vel_mult;
	}

    // The public methods below are the only ones required
    // by tools::ParticlesToLevelSet
    size_t size() const
	{
		return m_points->size();
	}

    // Position of particle in world space
    // This is required by ParticlesToLevelSet::rasterizeSpheres(*this, radius)
    void getPos(size_t n, openvdb::Vec3R& xyz) const
    {
        const auto p = (*m_points)[n];
        xyz[0] = p[0], xyz[1] = p[1], xyz[2] = p[2];
    }

    // Position and radius of particle in world space
    // This is required by ParticlesToLevelSet::rasterizeSpheres(*this)
    void getPosRad(size_t n, openvdb::Vec3R &xyz, openvdb::Real &rad) const
    {
        assert(m_has_radius);

		const auto &p = (*m_points)[n];
        xyz[0] = p[0], xyz[1] = p[1], xyz[2] = p[2];

        rad = m_rad_mult * m_scale->float_(n);
    }

    // Position, radius and velocity of particle in world space
    // This is required by ParticlesToLevelSet::rasterizeTrails
    void getPosRadVel(size_t n, openvdb::Vec3R& xyz,
                      openvdb::Real& rad, openvdb::Vec3R& vel) const
    {
        assert(m_has_velocity);

		const auto p = (*m_points)[n];
        xyz[0] = p[0], xyz[1] = p[1], xyz[2] = p[2];

        rad = m_has_radius ? m_rad_mult * m_scale->float_(n) : m_rad_mult;

		const auto &v = m_velocity->vec3(n);
        vel[0] = m_vel_mult * v[0], vel[1] = m_vel_mult * v[1], vel[2] = m_vel_mult * v[2];
    }

    // Required for attribute transfer
    void getAtt(size_t n, openvdb::Int32& att) const
	{
		att = openvdb::Int32(n);
	}
};

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

static inline auto create_point_mask_grid(const openvdb::math::Transform &xform,
                                          PointList *point_list)
{
    VDBPointList points(point_list);
    return openvdb::tools::createPointMask(points, xform);
}

/* ************************************************************************** */

static constexpr auto NOM_OPERATEUR = "OpenVDB From Particles";
static constexpr auto AIDE_OPERATEUR = "";

class NodeOpenVDBFromParticles : public OperateurOpenVDB {
public:
	NodeOpenVDBFromParticles(Noeud *noeud, const Context &contexte);

	const char *nom_entree(size_t index) override
	{
		if (index == 0) {
			return "Points";
		}

		return "reference";
	}

	const char *nom_sortie(size_t /*index*/) override { return "VDB"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void convert(openvdb::FloatGrid::Ptr grid, ParticleList &list);

	void execute(const Context &contexte, double temps) override;
	bool update_properties() override;
};

NodeOpenVDBFromParticles::NodeOpenVDBFromParticles(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(2);
	sorties(1);

	/* TODO: toggle for world space units */

	/* *********************** */

	add_prop("output_sdf", "Output Level Set", property_type::prop_bool);
	set_prop_default_value_bool(true);
	set_prop_tooltip("Compute a narrow-band signed distance / level set grid "
	                 "from the input points.");

	add_prop("sdf_name", "Distance VDB Name", property_type::prop_string);
	set_prop_default_value_string("surface");
	set_prop_tooltip("Distance grid name.");

	add_prop("output_fog", "Output Fog Volume", property_type::prop_bool);
	set_prop_tooltip("Compute a fog volume grid by remapping the level set "
	                 "volume to [0, 1] range.  The interior region is marked active "
		             "and set to one, the interior portion of the active narrow-band "
		             "is remapped to (0, 1] range to produce a smooth gradient and "
		             "all exterior regions are set to zero, marked inactive and pruned.");

	add_prop("fog_name", "Fog Volume Name", property_type::prop_string);
	set_prop_default_value_string("density");
	set_prop_tooltip("Fog volume grid name.");

	add_prop("output_mask", "Output Mask", property_type::prop_bool);
	set_prop_tooltip("Compute an alpha mask grid that can be used in subsequent "
	                 "filtering nodes to constrain smoothing operations and preserve "
		             "surface features.");

	add_prop("mask_name", "Mask VDB Name", property_type::prop_string);
	set_prop_default_value_string("boundingvolume");
	set_prop_tooltip("Mask grid name.");

	add_prop("bounding_limit", "Bounding Limit", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(0.25f);
	set_prop_tooltip("Percentage to increase and decrease the particle radius."
	                 " Used to define the maximum and minimum limit surfaces"
                     " for the alpha mask construction.");

	add_prop("merge_reference", "Merge With Reference VDB", property_type::prop_bool);

	add_prop("voxel_size", "Voxel Size", property_type::prop_float);
	set_prop_min_max(1e-5f, 5.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Uniform voxel edge length in world units.  "
	                 "Decrease the voxel size to increase the volume resolution.");

	add_prop("world_space", "Use World Space for Band", property_type::prop_bool);
	set_prop_tooltip("Switch between voxel and world space units for "
	                 "the half-band width.");

	add_prop("half_band", "Half-Band Voxels", property_type::prop_float);
	set_prop_min_max(1.0f, 10.0f);
	set_prop_default_value_float(3.0f);
	set_prop_tooltip("Half the width of the narrow band in voxel units. "
	                 "The default value 3 is recommended for level set volumes.");

	add_prop("half_band_ws", "Half-Band", property_type::prop_float);
	set_prop_min_max(1e-5f, 10.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Half the width of the narrow band in world space units.");

	/* --------------------- Conversion ------------------- */

	/* --------------------- Spheres ------------------- */
	add_prop("conversion", "Conversion", property_type::prop_bool);

	add_prop("particle_scale", "Particle Scale", property_type::prop_float);
	set_prop_min_max(0.0f, 2.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("The point scale attribute, which defines the world space "
	                 "particle radius, will be scaled by this.  A value of one is assumed "
	                 "if the scale attribute is missing.");

	add_prop("min_radius", "Minimum Radius", property_type::prop_float);
	set_prop_min_max(0.0f, 2.0f);
	set_prop_default_value_float(1.5f);
	set_prop_tooltip("Minimum radius in voxel units after scaling.  "
	                 "Particles smaller than this limit are ignored.");

	 add_prop("velocity_trails", "Velocity Trails", property_type::prop_bool);
	 set_prop_tooltip("Velocity trail splatting toggle.  Note this feature "
	                  "requires a velocity point attribute.");

	 add_prop("velocity_scale", "Velocity Scale", property_type::prop_float);
	 set_prop_min_max(0.0f, 1.0f);
	 set_prop_default_value_float(1.0f);
	 set_prop_tooltip("Scales the velocity point attribute 'v'.  Use "
	                  "this parameter to control the length of the velocity trails.");

	 add_prop("trail_resolution", "Trail Resolution", property_type::prop_float);
	 set_prop_min_max(0.2f, 2.0f);
	 set_prop_default_value_float(1.0f);
	 set_prop_tooltip("Defines the distance between particle instances.  Use this "
	                  "parameter to control aliasing and number of particle instances.");

	/* --------------------- Points ------------------- */

	add_prop("dilation", "Dilation", property_type::prop_int);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_int(1);
	set_prop_tooltip("Number of morphological dilation iterations "
	                 "used to expand the active voxel region.");

	add_prop("closing", "Closing", property_type::prop_int);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_int(1);
	set_prop_tooltip("Number of morphological closing iterations "
	                 "used to fill gaps in the active voxel region.");

	add_prop("smoothing", "Smoothing", property_type::prop_int);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_tooltip("Number of surface smoothing interations.");
}

bool NodeOpenVDBFromParticles::update_properties()
{
	set_prop_visible("sdf_name", eval_bool("output_sdf"));
    set_prop_visible("fog_name", eval_bool("output_fog"));

    const auto use_mask = eval_bool("output_mask");
    set_prop_visible("bounding_limit", use_mask);
    set_prop_visible("mask_name", use_mask);

	const auto has_ref_input = entree(1)->est_connectee();
    set_prop_visible("merge_reference", has_ref_input);
    set_prop_visible("voxel_size", !has_ref_input);

    const auto world_space_units = eval_bool("world_space");
    set_prop_visible("half_band", !world_space_units);
    set_prop_visible("half_band_ws", world_space_units);

    const auto use_trails =  eval_bool("velocity_trails");
    set_prop_visible("trail_resolution", use_trails);
    set_prop_visible("velocity_scale", use_trails);

	return true;
}

void NodeOpenVDBFromParticles::convert(openvdb::FloatGrid::Ptr grid, ParticleList &list)
{
	openvdb::tools::ParticlesToLevelSet<openvdb::FloatGrid> raster(*grid, nullptr);

	const auto min_radius = eval_float("min_radius");
	const auto trail_res = eval_float("trail_resolution");
	const auto velocity_trails = eval_bool("velocity_trails");

    raster.setRmin(min_radius);
    raster.setRmax(1e15f);

    if (velocity_trails && list.hasVelocity()) {
        raster.rasterizeTrails(list, trail_res);
    }
	else if (list.hasRadius()){
        raster.rasterizeSpheres(list);
    }
	else {
        raster.rasterizeSpheres(list, list.radiusMult());
    }

    /* Always prune to produce a valid narrow-band level set. */
    raster.finalize(true);

    if (raster.ignoredParticles()) {
		if (raster.getMinCount() > 0) {
			std::stringstream ss;

			ss << "Ignored " << raster.getMinCount()
			   << " small particle(s) (hint: change min_radius in Voxels)";

			this->ajoute_avertissement(ss.str().c_str());
		}

		if (raster.getMaxCount() > 0) {
			std::stringstream ss;

			ss << "Ignored " << raster.getMaxCount()
			   << " large particle(s) (hint: change min_radius in Voxels)";

			this->ajoute_avertissement(ss.str().c_str());
		}
    }
}

void NodeOpenVDBFromParticles::execute(const Context &contexte, double temps)
{
	openvdb::util::NullInterrupter boss;

	entree(0)->requiers_collection(m_collection, contexte, temps);

	auto voxel_size = eval_float("voxel_size");

	if (voxel_size < 1e-5f) {
        std::ostringstream ostr;
        ostr << "The voxel size ("<< voxel_size << ") is too small.";
		this->ajoute_avertissement(ostr.str());
        return;
    }

	const auto output_level_set = eval_bool("output_sdf");
	const auto output_fog_volume_grid = eval_bool("output_fog");
	const auto output_mask_grid = eval_bool("output_mask");
	const auto output_attribute_grid = false; // eval_bool("attrList"); /* TODO */

	if (!output_fog_volume_grid && !output_level_set && !output_attribute_grid) {
		 this->ajoute_avertissement("No output selected");
         return;
    }

	const auto part_scale = eval_float("particle_scale");
	const auto vel_scale = eval_float("velocity_scale");

	const auto do_sphere_conversion = eval_bool("conversion");

	float background = 0.0;

    if (eval_bool("world_space")) {
        background = eval_float("half_band_ws");
    }
	else {
		background = voxel_size * eval_float("half_band");
    }

	auto transform = openvdb::math::Transform::createLinearTransform(voxel_size);

	PrimitiveCollection tmp_collection(m_collection->factory());
	std::vector<Primitive *> to_destroy;

	auto reference_collection = entree(1)->requiers_collection(nullptr, contexte, temps);
	const auto reference = (reference_collection != nullptr);

	for (auto prim : primitive_iterator(m_collection, PrimPoints::id)) {
		auto points = static_cast<PrimPoints *>(prim);

		ParticleList paList(points, part_scale, vel_scale);

		openvdb::FloatGrid::Ptr outputGrid;

		if (reference) {
			auto it = primitive_iterator(reference_collection, VDBVolume::id);
			auto ref_prim = static_cast<VDBVolume *>(it.get());

			if (ref_prim != nullptr) {
				transform = ref_prim->getGrid().transform().copy();
				voxel_size = static_cast<float>(transform->voxelSize()[0]);

				const auto ref_is_level_set = is_level_set(ref_prim);

				/* Match the narrow band width. */
				if (ref_is_level_set && ref_prim->getGrid().type() == openvdb::FloatGrid::gridType()) {
					background =
                        openvdb::gridConstPtrCast<openvdb::FloatGrid>(ref_prim->getGridPtr())->background();

					this->ajoute_avertissement("Note: Matching reference level set half-band width "
					                  " and background value. (UI half-band parameter is ignored.)");
				}

				if (eval_int("merge_reference")) {
                    if (ref_is_level_set) {
                        outputGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(
                            ref_prim->getGrid().deepCopyGrid());

                        if (!outputGrid) {
							this->ajoute_avertissement("Cannot write into the selected"
							                  " reference grid because it is not a float grid.");
                        }
                    }
					else {
						this->ajoute_avertissement("Can only write directly into a level set grid.");
                    }
                }
			}
			else {
				this->ajoute_avertissement("Second input has no VDB primitives.");
			}
		}

		if (!outputGrid) {
            outputGrid = openvdb::FloatGrid::create(background);
        }

        outputGrid->setGridClass(openvdb::GRID_LEVEL_SET);
        outputGrid->setTransform(transform);

		int dilation = eval_int("dilation");
        int closing = eval_int("closing");
        int smoothing = eval_int("smoothing");
        int bandWidth = int(std::ceil(background / voxel_size));

		openvdb::MaskGrid::Ptr pointMaskGrid;

		if (do_sphere_conversion) {
            if ((eval_bool("velocity_trails")) && !paList.hasVelocity()) {
				this->ajoute_avertissement("Velocity trails require a velocity point attribute named 'velocity' of vec3 type.");
            }

            if (output_attribute_grid) {
				/* TODO. */
                //this->convertWithAttributes(outputGrid, paList, *ptGeo);
				this->ajoute_avertissement("Attribute transfer is not supported yet!");
            }
			else {
                this->convert(outputGrid, paList);
            }
        }
		else {
            pointMaskGrid = create_point_mask_grid(*transform, points->points());

            openvdb::FloatGrid::Ptr sdfGrid = openvdb::tools::topologyToLevelSet(
                *pointMaskGrid, bandWidth, closing, dilation, smoothing, &boss);

            openvdb::tools::csgUnion(*outputGrid, *sdfGrid);
        }

		if (output_mask_grid) {
            auto radiusScale = paList.radiusMult();
            auto offset = eval_float("bounding_limit");
            offset = std::min(std::max(offset, 0.0f), 1.0f); // clamp to zero-one range.

            auto maxGrid = openvdb::FloatGrid::create(background);
            maxGrid->setGridClass(openvdb::GRID_LEVEL_SET);
            maxGrid->setTransform(transform->copy());

            auto minGrid = openvdb::FloatGrid::create(background);
            minGrid->setGridClass(openvdb::GRID_LEVEL_SET);
            minGrid->setTransform(transform->copy());

            if (offset > 0.0f) {
                if (do_sphere_conversion) {
                    paList.radiusMult() = radiusScale * (1.0 + offset);
                    this->convert(maxGrid, paList);

                    paList.radiusMult() = radiusScale * (1.0 - offset);
                    this->convert(minGrid, paList);
                }
				else {
                    if (!pointMaskGrid) {
                        pointMaskGrid = create_point_mask_grid(*transform, points->points());
                    }

                    openvdb::Real dx = openvdb::Real(std::min(dilation, 1));
                    int increase = int(std::ceil(dx * (1.0 + offset)));
                    int decrease = int(dx * (1.0 - offset));

                    maxGrid = openvdb::tools::topologyToLevelSet(
                        *pointMaskGrid, bandWidth, closing, increase, smoothing, &boss);

                    minGrid = openvdb::tools::topologyToLevelSet(
                        *pointMaskGrid, bandWidth, closing, decrease, smoothing, &boss);
                }
            }

            openvdb::tools::csgDifference(*maxGrid, *minGrid);
            openvdb::tools::sdfToFogVolume(*maxGrid);

			const auto name = eval_string("mask_name");
            maxGrid->setName(name);

            build_vdb_prim(&tmp_collection, maxGrid);
        }

		if (output_level_set) {
			const auto name = eval_string("sdf_name");
            outputGrid->setName(name);

			build_vdb_prim(&tmp_collection, outputGrid);
        }

		if (output_fog_volume_grid) {
			// Only duplicate the output grid if both distance
            // and fog volume grids are exported.
            if (output_level_set) {
                outputGrid = outputGrid->deepCopy();
            }

			openvdb::tools::sdfToFogVolume(*outputGrid);

			const auto name = eval_string("fog_name");
            outputGrid->setName(name);

			build_vdb_prim(&tmp_collection, outputGrid);
		}
	}

	m_collection->destroy(to_destroy);
	m_collection->merge_collection(tmp_collection);
}

/* ************************************************************************** */

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeOpenVDBFromParticles>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
