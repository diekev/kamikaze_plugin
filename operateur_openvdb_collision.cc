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
 */

static constexpr auto NOM_OPERATEUR = "Collision OpenVDB";
static constexpr auto AIDE_OPERATEUR = "Détection de collision en utilisant OpenVDB.";

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
				auto velocite = attr_vel->vec3(i) + gravite;
				const auto velocite_etape = velocite / 5.0f;

				for (auto e = 1; e <= 5; ++e) {
					auto &point = (*points)[i];
					point += velocite_etape;

					/* Calcul la position en espace objet. */
					const auto pos = nuage_points->matrix() * point;
					const auto pos_vdb = openvdb::Vec3f(pos.x, pos.y, pos.z);

					/* Échantillone le volume à la position de la particule. */
					auto valeur = sampler.wsSample(pos_vdb);

					if (valeur > 0.0f) {
						/* La particule est en dehors de l'objet. */
					}
					else if (valeur < 0.0f) {
						/* La particule est à l'intérieur de l'objet. */

						/* Calculer le gradient de la collision. */
						stencil.moveTo(pos_vdb);
						auto cpt = stencil.cpt();

						/* Replacer la particule sur la surface selon le gradient. */
						point[0] = cpt[0];
						point[1] = cpt[1];
						point[2] = cpt[2];

						/* Répondre à la collision. */
						velocite = -velocite;

						/* Une fois que la collision a été établi, on peut
						 * passer à la particule suivante. */
						break;
					}
					else {
						/* La particule est sur l'objet. */

						/* Répondre à la collision. */
						velocite = -velocite;

						/* Une fois que la collision a été établi, on peut
						 * passer à la particule suivante. */
						break;
					}
				}

				attr_vel->vec3(i, velocite);
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
