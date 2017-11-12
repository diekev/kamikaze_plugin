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

#include <openvdb/tools/VectorTransformer.h>

#include "util_openvdb_process.h"
#include "volumebase.h"

static constexpr auto NOM_OPERATEUR = "OpenVDB Transform";
static constexpr auto AIDE_OPERATEUR = "";

class NodeOpenVDBTransform : public OperateurOpenVDB {
public:
	NodeOpenVDBTransform(Noeud *noeud, const Context &contexte);

	const char *nom_entree(size_t /*index*/) override { return "VDB"; }
	const char *nom_sortie(size_t /*index*/) override { return "VDB"; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;
};

NodeOpenVDBTransform::NodeOpenVDBTransform(Noeud *noeud, const Context &contexte)
    : OperateurOpenVDB(noeud, contexte)
{
	entrees(1);
	sorties(1);

	add_prop("translate", "Translate", property_type::prop_vec3);
	set_prop_min_max(-10.0f, 10.0f);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});

	add_prop("rotate", "Rotate", property_type::prop_vec3);
	set_prop_min_max(0.0f, 360.0f);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});

	add_prop("scale", "Scale", property_type::prop_vec3);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_default_value_vec3(glm::vec3{1.0f, 1.0f, 1.0f});

	add_prop("pivot", "Pivot", property_type::prop_vec3);
	set_prop_min_max(-10.0f, 10.0f);
	set_prop_default_value_vec3(glm::vec3{0.0f, 0.0f, 0.0f});

	add_prop("uniform_scale", "Uniform Scale", property_type::prop_float);
	set_prop_min_max(0.0f, 1000.0f);
	set_prop_default_value_float(1.0f);

	add_prop("invert", "Invert Transformation", property_type::prop_bool);

	add_prop("xform_vectors", "Transform Vectors", property_type::prop_bool);
	set_prop_tooltip("Apply the transformation to the voxels of vector-valued"
	                 " grids, according to their vector type.");
}

/* Functor to apply a transform to the voxel values of vector-valued grids */
struct VectorTransformOp {
    openvdb::Mat4d mat;

    VectorTransformOp(const openvdb::Mat4d &m)
	    : mat(m)
	{}

    template<typename GridT>
	void operator()(GridT& grid) const
    {
        openvdb::tools::transformVectors(grid, mat);
    }
};

void NodeOpenVDBTransform::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	const auto translate = eval_vec3("translate");
    const auto rotate = eval_vec3("rotate");
    const auto pivot = eval_vec3("pivot");
	const auto invert = eval_bool("invert");
	const auto xform_vector = eval_bool("xform_vectors");
	const auto uniform_scale = eval_float("uniform_scale");
	const auto scale = eval_vec3("scale") * uniform_scale;

	constexpr auto deg2rad = M_PI / 180.0f;

	openvdb::Mat4R mat(openvdb::Mat4R::identity());
    mat.preTranslate(openvdb::math::Vec3s(&pivot[0]));
    mat.preRotate(openvdb::math::X_AXIS, deg2rad * rotate[0]);
    mat.preRotate(openvdb::math::Y_AXIS, deg2rad * rotate[1]);
    mat.preRotate(openvdb::math::Z_AXIS, deg2rad * rotate[2]);
    mat.preScale(openvdb::math::Vec3s(&scale[0]));
    mat.preTranslate(-openvdb::math::Vec3s(&pivot[0]));
    mat.preTranslate(openvdb::math::Vec3s(&translate[0]));

	if (invert) {
		mat.inverse();
	}

	const VectorTransformOp xformOp(mat);

	// Construct an affine map.
    openvdb::math::AffineMap map(mat);

	for (auto &prim : primitive_iterator(this->m_collection, VDBVolume::id)) {
		auto vdb_prim = static_cast<VDBVolume *>(prim);
		auto grid = vdb_prim->getGridPtr();

		// Merge the transform's current affine representation with the new affine map.
	    openvdb::math::AffineMap::Ptr compound(
	        new openvdb::math::AffineMap(*grid->transform().baseMap()->getAffineMap(), map));

		grid->setTransform(openvdb::math::Transform::Ptr(new openvdb::math::Transform(openvdb::math::simplify(compound))));

		if (xform_vector &&
		    is_vector_grid(vdb_prim) &&
		    grid->isInWorldSpace() &&
		    grid->getVectorType() != openvdb::VEC_INVARIANT)
		{
			process_grid_vector(*grid, vdb_prim->storage(), xformOp);
		}

		vdb_prim->setGrid(grid);
	}
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeOpenVDBTransform>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "OpenVDB"));
}

}
