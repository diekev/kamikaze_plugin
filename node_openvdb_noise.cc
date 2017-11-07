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
#include <kamikaze/noise.h>

#include <openvdb/math/Operators.h>
#include <openvdb/tools/Interpolation.h> /* for box sampler */

#include "util_openvdb_process.h"
#include "volumebase.h"

/* ************************************************************************** */

enum {
	NOISE_STRAIGHT     = 0,
	NOISE_ABSOLUTE     = 1,
	NOISE_ABSOLUTE_INV = 2,
};

enum {
	MASK_LESS        = 0,
	MASK_GREATER     = 1,
	MASK_GREATER_NRM = 2,
	MASK_FREQ_MULT   = 3,
};

/* ************************************************************************** */

namespace {

class FractalBoltzmanGenerator {
	int m_octaves;
    int m_noise_mode;
    float m_freq;
    float m_amp;
    float m_gain;
    float m_lacunarity;
    float m_roughness;

public:
    FractalBoltzmanGenerator(float freq, float amp, int octaves, float gain,
	                         float lacunarity, float roughness, int mode)
	    : m_octaves(octaves)
	    , m_noise_mode(mode)
	    , m_freq(freq)
	    , m_amp(amp)
	    , m_gain(gain)
	    , m_lacunarity(lacunarity)
	    , m_roughness(roughness)
    {}

    // produce the noise as float
    float noise(openvdb::Vec3R point, float freqMult = 1.0f) const
    {
        float signal;
        float result = 0.0f;
        float curamp = m_amp;
        float curfreq = m_freq * freqMult;

        for (int n = 0; n <= m_octaves; n++) {
            point = (point * curfreq);

            // generate noise in the [-1,1] range
            signal = 2.0f * simplex_noise_3d(point[0], point[1], point[2]) - 1.0f;

            if (m_noise_mode > NOISE_STRAIGHT) {
                signal = openvdb::math::Pow(openvdb::math::Abs(signal), m_gain);
            }

            result  += (signal * curamp);
            curfreq = m_lacunarity;
            curamp *= m_roughness;
        }

        if (m_noise_mode == NOISE_ABSOLUTE) {
            result = -result;
        }

        return result;
    }
};

struct NoiseSettings {
    NoiseSettings() = default;

    int mask_mode = MASK_LESS;
    float offset = 0.0f;
	float threshold = 0.0f;
	float falloff = 0.0f;
    openvdb::Vec3R noise_offset = openvdb::Vec3R(0.0f, 0.0f, 0.0f);
};

}

/* ************************************************************************** */

static constexpr auto NOM_OPERATEUR = "OpenVDB Noise";
static constexpr auto AIDE_OPERATEUR = "";

class NodeNoise : public OperateurOpenVDB {
public:
	NodeNoise(Noeud *noeud, const Context &contexte);

	const char *nom_entree(size_t index) override
	{
		if (index == 0) {
			return "input grids";
		}

		return "mask (optional)";
	}

	const char *nom_sortie(size_t /*index*/) override { return "output"; }

	bool update_properties() override;

	void execute(const Context &contexte, double temps) override;

	template<typename GridType>
	void apply_noise(openvdb::GridBase &grid,
	                 const FractalBoltzmanGenerator& fbGenerator,
	                 const NoiseSettings& settings,
	                 const openvdb::GridBase *mask) const;
};

NodeNoise::NodeNoise(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(2);
	sorties(1);

	/* ------------------------- Noise parameters. -------------------------- */

	add_prop("amplitude", "Amplitude", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(1.0f);

	add_prop("frequency", "Frequency", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(1.0f);

	add_prop("octaves", "Octaves", property_type::prop_int);
	set_prop_min_max(0, 10);
	set_prop_default_value_int(0);

	add_prop("lacunarity", "Lacunarity", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(2.0f);

	add_prop("gain", "Gain", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(1.0f);

	add_prop("roughness", "Roughness", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(1.0f);

	add_prop("surface_offset", "Surface Offset", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(1.0f);

	add_prop("noise_offset", "Noise Offset", property_type::prop_vec3);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});

	EnumProperty noise_enum;
	noise_enum.insert("Straight", NOISE_STRAIGHT);
	noise_enum.insert("Absolute", NOISE_ABSOLUTE);
	noise_enum.insert("Inverse Absolute", NOISE_ABSOLUTE_INV);

	add_prop("noise_mode", "Noise Mode", property_type::prop_enum);
	set_prop_enum_values(noise_enum);

	/* -------------------------- Mask parameters. -------------------------- */

	EnumProperty mask_enum;
	mask_enum.insert("No noise if mask < threshold", MASK_LESS);
	mask_enum.insert("No noise if mask > threshold", MASK_GREATER);
	mask_enum.insert("No noise if mask > threshold & normals align", MASK_GREATER_NRM);
	mask_enum.insert("Use mask as frequency multiplier", MASK_FREQ_MULT);

	add_prop("mask_mode", "Mask Mode", property_type::prop_enum);
	set_prop_enum_values(mask_enum);

	add_prop("threshold", "Threshold", property_type::prop_float);
	set_prop_min_max(0.0f, 1.0f);
	set_prop_default_value_float(0.0f);

	add_prop("falloff", "Falloff", property_type::prop_float);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_float(0.0f);
}

bool NodeNoise::update_properties()
{
	const auto has_mask = entree(1)->est_connectee();

	set_prop_visible("mask_mode", has_mask);
	set_prop_visible("threshold", has_mask);
	set_prop_visible("falloff", has_mask);

	return true;
}

void NodeNoise::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	/* ------- Evaluate the FractalBoltzman noise parameters from UI. ------- */

    FractalBoltzmanGenerator fbGenerator(eval_float("frequency"),
                                         eval_float("amplitude"),
                                         eval_int("octaves"),
                                         eval_float("gain"),
                                         eval_float("lacunarity"),
                                         eval_float("roughness"),
                                         eval_int("noise_mode"));

	NoiseSettings settings;

	/* --------------- Evaluate parameter for blending noise. --------------- */

    settings.offset = eval_float("surface_offset");

	const auto noise_offset = eval_vec3("noise_offset");

    settings.noise_offset = openvdb::Vec3R(noise_offset[0], noise_offset[1], noise_offset[2]);

	/* ------------------------- Get the mask grid. ------------------------- */

    const openvdb::GridBase *maskGrid = nullptr;
	const auto refGdp = entree(1)->requiers_collection(nullptr, contexte, temps);

    if (refGdp != nullptr) {
		for (auto &prim : primitive_iterator(refGdp, VDBVolume::id)) {
			auto vdbPrim = static_cast<VDBVolume *>(prim);

			/* If we already have a mask grid, report an error. */
			if (maskGrid != nullptr) {
				std::ostringstream ostr;
	            ostr << "Found more than one grid in the mask group; the first grid will be used.";
				this->ajoute_avertissement(ostr.str());
				break;
			}

			if (vdbPrim != nullptr) {
	            settings.mask_mode = eval_int("mask_mode");
	            settings.threshold = eval_float("threshold");
	            settings.falloff = eval_float("falloff");

	            maskGrid = &(vdbPrim->getGrid());
	        }
		}
    }

	/* ---------------------- Process the input grids. ---------------------- */

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdbPrim = static_cast<VDBVolume *>(prim);

		if (vdbPrim->storage() == GRID_STORAGE_FLOAT) {
            apply_noise<openvdb::ScalarGrid>(vdbPrim->getGrid(), fbGenerator, settings, maskGrid);
        }
		else if (vdbPrim->storage() == GRID_STORAGE_DOUBLE) {
            apply_noise<openvdb::DoubleGrid>(vdbPrim->getGrid(), fbGenerator, settings, maskGrid);
        }
		else {
            std::stringstream ss;
            ss << "VDB primitive " << vdbPrim->name()
                << " was skipped because it is not a scalar grid.";
			this->ajoute_avertissement(ss.str());
            continue;
        }
	}
}

template<typename GridType>
void NodeNoise::apply_noise(openvdb::GridBase &grid,
                            const FractalBoltzmanGenerator& fbGenerator,
                            const NoiseSettings& settings,
                            const openvdb::GridBase *mask) const
{
    /* Use second order finite difference. */
	using Gradient = openvdb::math::Gradient<openvdb::math::GenericMap, openvdb::math::CD_2ND>;
    using CPT = openvdb::math::CPT<openvdb::math::GenericMap, openvdb::math::CD_2ND>;
    using StencilType = openvdb::math::SecondOrderDenseStencil<GridType>;

	using TreeType = typename GridType::TreeType;
	using Vec3Type = openvdb::math::Vec3<typename TreeType::ValueType>;

    /* Down cast the generic pointer to the output grid. */
    auto &outGrid = VDB_grid_cast<GridType>(grid);

    const auto &xform = grid.transform();

    /* Create a stencil. */
    StencilType stencil(outGrid); /* uses its own grid accessor */

    /* scratch variables */
    typename GridType::ValueType result; /* result - use mask as frequency multiplier */
    openvdb::Vec3R voxelPt; /* voxel coordinates */
    openvdb::Vec3R worldPt; /* world coordinates */
    float noise, alpha;

    /* The use of the GenericMap is a performance compromise because the
     * GenericMap holds a base class pointer. This should be optimized by
     * resolving the acutal map type */
    openvdb::math::GenericMap map(grid);

    if (!mask) {
        alpha = 1.0f;

        for (typename GridType::ValueOnIter v = outGrid.beginValueOn(); v; ++v) {
            stencil.moveTo(v);
            worldPt = xform.indexToWorld(CPT::result(map, stencil) + settings.noise_offset);
            noise = fbGenerator.noise(worldPt);
            v.setValue(*v + alpha * (noise - settings.offset));
        }

        return;
    }

    /* Down cast the generic pointer to the mask grid. */
    const auto *maskGrid = VDB_grid_cast<GridType>(mask);
    const auto &maskXform = mask->transform();

    switch (settings.mask_mode) {
        case MASK_LESS: /* No noise if mask < threshold */
        {
            for (typename GridType::ValueOnIter v = outGrid.beginValueOn(); v; ++v) {
                openvdb::Coord ijk = v.getCoord();
                stencil.moveTo(ijk); /* in voxel units */

                worldPt = xform.indexToWorld(ijk);
                voxelPt = maskXform.worldToIndex(worldPt);
                openvdb::tools::BoxSampler::sample<TreeType>(maskGrid->tree(), voxelPt, result);

                /* apply threshold */
                if (result < settings.threshold) {
                    continue; //next voxel
                }

                alpha = static_cast<float>(result >= settings.threshold + settings.falloff ?
                    1.0f : (result - settings.threshold) / settings.falloff);

                worldPt = xform.indexToWorld(CPT::result(map, stencil) + settings.noise_offset);
                noise = fbGenerator.noise(worldPt);
                v.setValue(*v + alpha * (noise - settings.offset));
            }

			break;
        }
        case MASK_GREATER: /* No noise if mask > threshold */
        {
            for (typename GridType::ValueOnIter v = outGrid.beginValueOn(); v; ++v) {
                openvdb::Coord ijk = v.getCoord();
                stencil.moveTo(ijk); /* in voxel units */

                worldPt = xform.indexToWorld(ijk);
                voxelPt = maskXform.worldToIndex(worldPt);
                openvdb::tools::BoxSampler::sample<TreeType>(maskGrid->tree(), voxelPt, result);

                /* apply threshold */
                if (result > settings.threshold) {
                    continue; //next voxel
                }
                alpha = static_cast<float>(result <= settings.threshold - settings.falloff ?
                    1.0f : (settings.threshold - result) / settings.falloff);

                worldPt = xform.indexToWorld(CPT::result(map, stencil) + settings.noise_offset);
                noise = fbGenerator.noise(worldPt);
                v.setValue(*v + alpha * (noise - settings.offset));
            }

			break;
        }
        case MASK_GREATER_NRM: /* No noise if mask < threshold & normals align */
        {
            StencilType maskStencil(*maskGrid);
            for (typename GridType::ValueOnIter v = outGrid.beginValueOn(); v; ++v) {
                openvdb::Coord ijk = v.getCoord();
                stencil.moveTo(ijk); /* in voxel units */

                worldPt = xform.indexToWorld(ijk);
                voxelPt = maskXform.worldToIndex(worldPt);
                openvdb::tools::BoxSampler::sample<TreeType>(maskGrid->tree(), voxelPt, result);

                /* for the gradient of the maskGrid */
                openvdb::Coord mask_ijk(
                    static_cast<int>(voxelPt[0]),
                    static_cast<int>(voxelPt[1]),
                    static_cast<int>(voxelPt[2]));
                maskStencil.moveTo(mask_ijk);
                /* normal alignment */
                Vec3Type grid_grad = Gradient::result(map, stencil);
                Vec3Type mask_grad = Gradient::result(map, maskStencil);
                const double c = openvdb::math::Abs(grid_grad.dot(mask_grad));

                if (result > settings.threshold && c > 0.9) continue;//next voxel
                alpha = static_cast<float>(result <= settings.threshold - settings.falloff ?
                    1.0f : (settings.threshold - result) / settings.falloff);

                worldPt = xform.indexToWorld(CPT::result(map, stencil) + settings.noise_offset);
                noise = fbGenerator.noise(worldPt);
                v.setValue(*v + alpha * (noise - settings.offset));
            }

			break;
        }
        case MASK_FREQ_MULT: /* Use mask as frequency multiplier */
        {
            alpha = 1.0f;
            for (typename GridType::ValueOnIter v = outGrid.beginValueOn(); v; ++v) {
                openvdb::Coord ijk = v.getCoord();
                stencil.moveTo(ijk); /* in voxel units */

                worldPt = xform.indexToWorld(ijk);
                voxelPt = maskXform.worldToIndex(worldPt);
                openvdb::tools::BoxSampler::sample<TreeType>(maskGrid->tree(), voxelPt, result);

                worldPt = xform.indexToWorld(CPT::result(map, stencil) + settings.noise_offset);
                /* Use result of sample as frequency multiplier. */
                noise = fbGenerator.noise(worldPt, static_cast<float>(result));
                v.setValue(*v + alpha * (noise - settings.offset));
            }

			break;
        }
        default: /* should never get here */
            throw std::runtime_error("internal error in mode selection");
    }
}

/* ************************************************************************** */

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeNoise>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
