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

#include <kamikaze/primitive.h>
#include <kamikaze/prim_points.h>
#include <kamikaze/utils_glm.h>

#include <openvdb/tools/Interpolation.h>
#include <openvdb/math/Stencils.h>

#include "volumebase.h"

/**
 * Basé sur « Levelsets in production: Spider-Man 3 » :
 * http://library.imageworks.com/pdfs/imageworks-library-levelsets-in-production-spider-man3.pdf
 *
 * Algorithme de résolution des particules basé sur
 * « Physically Based Modeling: Principles and Practice » :
 * https://www.cs.cmu.edu/~baraff/sigcourse/notesc.pdf
 */

static constexpr auto NOM_OPERATEUR = "Collision OpenVDB";
static constexpr auto AIDE_OPERATEUR = "Détection de collision en utilisant OpenVDB.";

enum {
	COLLISION_ELASTIQUE = 0,
	COLLISION_NON_ELASTIQUE = 1,
};

class OperateurOpenVDBCollision : public OperateurOpenVDB {
	glm::vec3 m_gravite = glm::vec3{0.0f, -9.80665f, 0.0f};
	PrimitiveCollection *m_collection_original = nullptr;
	PrimitiveCollection *m_derniere_collection = nullptr;
	int m_image_debut = 0;

public:
	OperateurOpenVDBCollision(Noeud *noeud, const Context &contexte)
		: OperateurOpenVDB(noeud, contexte)
	{
		entrees(2);
		sorties(1);

		add_prop("sous_etapes", "Sous-étapes", property_type::prop_int);
		set_prop_default_value_int(5);
		set_prop_min_max(1, 20);
		set_prop_tooltip("Le nombre de sous étape à prendre pour chaque évaluation.");

		EnumProperty enum_prop;
		enum_prop.insert("Élastique", COLLISION_ELASTIQUE);
		enum_prop.insert("Non-élastique", COLLISION_NON_ELASTIQUE);

		add_prop("type_collision", "Type collision", property_type::prop_enum);
		set_prop_enum_values(enum_prop);

		add_prop("coef_restitution", "Coefficient Restitution", property_type::prop_float);
		set_prop_default_value_float(0.5f);
		set_prop_min_max(0.0f, 1.0f);
	}

	~OperateurOpenVDBCollision()
	{
		delete m_collection_original;
		delete m_derniere_collection;
	}

	const char *nom_entree(size_t index) override
	{
		if (index == 0) {
			return "Particules";
		}

		return "Obstacle";
	}

	const char *nom_sortie(size_t /*index*/) override
	{
		return "Sortie";
	}

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override
	{
		if (temps == m_image_debut) {
			m_collection->free_all();
			entree(0)->requiers_collection(m_collection, contexte, temps);

			delete m_collection_original;
			m_collection_original = m_collection->copy();
		}
		else {
			m_collection = m_derniere_collection->copy();
		}

		execute_algorithme(contexte, temps);

		/* Sauvegarde la collection */
		delete m_derniere_collection;
		m_derniere_collection = m_collection->copy();
	}

	void execute_algorithme(const Context &contexte, double temps)
	{
		/* À FAIRE : passe le temps par image en paramètre. */
		const auto temps_par_image = 1.0f / 24.0f;
		const auto gravite = m_gravite * temps_par_image;
		const auto sous_etapes = eval_int("sous_etapes");
		const auto echelle_sous_etapes = 1.0f / sous_etapes;
		const auto type_collision = eval_enum("type_collision");
		const auto coefficient_restitution = eval_float("coef_restitution");

		auto collection_obstacle = entree(1)->requiers_collection(nullptr, contexte, temps);

		if (collection_obstacle == nullptr) {
			this->ajoute_avertissement("La prise d'obstacle n'est pas connectée !");
			return;
		}

		VDBVolume *grille_obstacle = nullptr;

		for (auto prim : primitive_iterator(collection_obstacle, VDBVolume::id)) {
			grille_obstacle = static_cast<VDBVolume *>(prim);
			break;
		}

		if (grille_obstacle == nullptr) {
			this->ajoute_avertissement("Il n'y a pas de volume dans la collection d'obstacle !");
			return;
		}

		auto grille_vdb = openvdb::gridPtrCast<openvdb::FloatGrid>(grille_obstacle->getGridPtr());
		openvdb::FloatGrid::Accessor accessor = grille_vdb->getAccessor();
		openvdb::math::Transform transformation = grille_vdb->transform();

		auto sampler = openvdb::tools::GridSampler<openvdb::tree::ValueAccessor<openvdb::FloatTree>, openvdb::tools::PointSampler>(accessor, transformation);
		auto stencil = openvdb::math::GradStencil<openvdb::FloatGrid>(*grille_vdb);

		for (Primitive *prim : primitive_iterator(m_collection, PrimPoints::id)) {
			auto nuage_points = static_cast<PrimPoints *>(prim);
			auto points = nuage_points->points();
			auto nombre_points = points->size();

			auto attr_vel = nuage_points->add_attribute("velocité", ATTR_TYPE_VEC3, nombre_points);

			for (auto i = 0ul; i < nombre_points; ++i) {
				auto &point = (*points)[i];
				auto velocite = attr_vel->vec3(i) + gravite;
				auto velocite_etape = velocite * echelle_sous_etapes;
				auto velocite_final = glm::vec3{0.0f, 0.0f, 0.0f};

				for (auto e = 1; e <= sous_etapes; ++e) {
					point += velocite_etape;

					/* Calcul la position en espace objet. */
					const auto pos = nuage_points->matrix() * point;
					const auto pos_vdb = openvdb::Vec3f(pos.x, pos.y, pos.z);

					/* Échantillone le volume à la position de la particule. */
					auto valeur = sampler.wsSample(pos_vdb);

					if (valeur > 0.0f) {
						/* La particule est en dehors de l'objet. */
						continue;
					}

					/* La particule est sur ou à l'intérieur de l'objet. */

					/* Calculer le gradient de la collision. */
					stencil.moveTo(pos_vdb);
					auto gradient = stencil.gradient();

					/* Replacer la particule sur la surface selon le gradient. */
					point[0] -= gradient[0];
					point[1] -= gradient[1];
					point[2] -= gradient[2];

					/* Répondre à la collision. */

					/* Trouve la composante normale de la vélocité :
					 * nv = (N.v)*v, où N est la normal de la collision. */
					stencil.moveTo(pos_vdb - gradient);
					gradient = stencil.gradient();

					auto N = glm::vec3(gradient[0], gradient[1], gradient[2]);
					auto nv = glm::dot(N, velocite_etape) * velocite_etape;

					/* Trouve la composante tangentielle de la vélocité :
					 * tn = v - nv */
					auto tn = velocite_etape - nv;

					if (type_collision == COLLISION_ELASTIQUE) {
						/* Annule la composante normale de la vélocité. */
						velocite_etape = tn - nv;
					}
					else {
						/* Multiplie la composante normale de la vélocité par le
						 * coefficient de restitution. */
						velocite_etape = tn + nv * -coefficient_restitution;
					}

					velocite_final += velocite_etape;
				}

				attr_vel->vec3(i, velocite_final);
			}
		}
	}
};

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<OperateurOpenVDBCollision>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "Physique"));
}

}
