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

#include <kamikaze/mesh.h>
#include <kamikaze/operateur.h>
#include <kamikaze/prim_points.h>

#include <kamikaze/outils/mathématiques.h>

#include <map>
#include <random>
#include <sstream>
#include <stack>

/**
 * "Dart Throwing on Surfaces" (Cline et al. 2009)
 * http://peterwonka.net/Publications/pdfs/2009.EGSR.Cline.PoissonSamplingOnSurfaces.pdf
 */

/* La densité de l'arrangement de cercles ayant la plus grande densité, selon
 * Lagrange. C'est-à-dire le pourcentage d'aire qu'occuperait un arrangement de
 * cercle étant le plus compacte. */
static constexpr auto DENSITE_CERCLE = 0.9068996821171089;

static constexpr auto NOMBRE_BOITE = 64;

static const char *NOM_OPERATEUR = "Tirage de flèche";
static const char *AIDE_OPERATEUR =
		"Crée des points sur une surface en l'échantillonnant par tirage de flèche.";

/* ************************************************************************** */

struct HachageSpatial {
	std::unordered_map<std::size_t, std::vector<glm::vec3>> m_tableau;

	/**
	 * La taille maximum recommandée par la publication de Cline et al. est de
	 * 20 000. Cependant, les fonctions de hachage marche mieux quand la taille
	 * est un nombre premier ("Introduction to Algorithms", ISBN 0-262-03141-8),
	 * donc nous prenons le nombre premier le plus proche de 20 000.
	 */
	static constexpr auto TAILLE_MAX = 19997;

	/**
	 * Fonction de hachage repris de "Optimized Spatial Hashing for Collision
	 * Detection of Deformable Objects"
	 * http://www.beosil.com/download/CollisionDetectionHashing_VMV03.pdf
	 *
	 * Pour calculer l'empreinte d'une position, nous considérons la partie
	 * entière de celle-ci. Par exemple, le vecteur position <0.1, 0.2, 0.3>
	 * deviendra <0, 0, 0> ; de même pour le vecteur <0.4, 0.5, 0.6>. Ainsi,
	 * toutes les positions se trouvant entre <0, 0, 0> et
	 * <0.99.., 0.99.., 0.99..> seront dans la même alvéole.
	 */
	std::size_t fonction_empreinte(const glm::vec3 &position);

	/**
	 * Ajoute la posistion spécifiée dans le vecteur des positions ayant la même
	 * empreinte que celle-ci.
	 */
	void ajoute(const glm::vec3 &position);

	/**
	 * Retourne un vecteur contenant les positions ayant la même empreinte que
	 * la position passée en paramètre.
	 */
	const std::vector<glm::vec3> &particules(const glm::vec3 &position);

	/**
	 * Retourne le nombre d'alvéoles présentes dans la table de hachage.
	 */
	size_t taille() const;
};

std::size_t HachageSpatial::fonction_empreinte(const glm::vec3 &position)
{
	return static_cast<std::size_t>(
				static_cast<int>(position.x) * 73856093
				^ static_cast<int>(position.y) * 19349663
				^ static_cast<int>(position.z) * 83492791) % TAILLE_MAX;
}

void HachageSpatial::ajoute(const glm::vec3 &position)
{
	const auto empreinte = fonction_empreinte(position);
	m_tableau[empreinte].push_back(position);
}

const std::vector<glm::vec3> &HachageSpatial::particules(const glm::vec3 &position)
{
	const auto empreinte = fonction_empreinte(position);
	return m_tableau[empreinte];
}

size_t HachageSpatial::taille() const
{
	return m_tableau.size();
}

/* ************************************************************************** */

struct Triangle {
	glm::vec3 v0 = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 v1 = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 v2 = glm::vec3(0.0f, 0.0f, 0.0f);
	float aire = 0.0f;
	Triangle *precedent = nullptr, *suivant = nullptr;

	Triangle() = default;

	Triangle(const glm::vec3 &v_0, const glm::vec3 &v_1, const glm::vec3 &v_2)
		: Triangle()
	{
		v0 = v_0;
		v1 = v_1;
		v2 = v_2;
	}
};

static float calcule_aire(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2)
{
	const auto c1 = v1 - v0;
	const auto c2 = v2 - v0;

	return glm::length(glm::cross(c1, c2)) * 0.5f;
}

static float calcule_aire(const Triangle &triangle)
{
	return calcule_aire(triangle.v0, triangle.v1, triangle.v2);
}

class ListeTriangle {
	Triangle *m_premier_triangle = nullptr;
	Triangle *m_dernier_triangle = nullptr;

public:
	ListeTriangle() = default;
	~ListeTriangle() = default;

	Triangle *ajoute(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2)
	{
		Triangle *triangle = new Triangle(v0, v1, v2);
		triangle->aire = calcule_aire(*triangle);
		triangle->precedent = nullptr;
		triangle->suivant = nullptr;

		if (m_premier_triangle == nullptr) {
			m_premier_triangle = triangle;
		}
		else {
			triangle->precedent = m_dernier_triangle;
			m_dernier_triangle->suivant = triangle;
		}

		m_dernier_triangle = triangle;

		return triangle;
	}

	void enleve(Triangle *triangle)
	{
		if (triangle->precedent) {
			triangle->precedent->suivant = triangle->suivant;
		}

		if (triangle->suivant) {
			triangle->suivant->precedent = triangle->precedent;
		}

		if (triangle == m_premier_triangle) {
			m_premier_triangle = triangle->suivant;

			if (m_premier_triangle) {
				m_premier_triangle->precedent = nullptr;
			}
		}

		if (triangle == m_dernier_triangle) {
			m_dernier_triangle = triangle->precedent;

			if (m_dernier_triangle) {
				m_dernier_triangle->precedent = nullptr;
			}
		}

		delete triangle;
	}

	Triangle *premier_triangle()
	{
		return m_premier_triangle;
	}

	bool vide() const
	{
		return m_premier_triangle == nullptr;
	}
};

struct BoiteTriangle {
	float aire_minimum = std::numeric_limits<float>::max();
	float aire_maximum = 0.0f; // = 2 * aire_minimum
	float aire_totale = 0.0f;

	ListeTriangle triangles;
};

void ajoute_triangle_boite(BoiteTriangle *boite, const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2)
{
	auto triangle = boite->triangles.ajoute(v0, v1, v2);
	boite->aire_minimum = std::min(boite->aire_minimum, triangle->aire);
	boite->aire_maximum = 2 * boite->aire_minimum;
	boite->aire_totale += triangle->aire;
}

bool verifie_distance_minimal(HachageSpatial &hachage_spatial, const glm::vec3 &point, float distance)
{
	const auto points = hachage_spatial.particules(point);

	for (auto p = 0ul; p < points.size(); ++p) {
		if (glm::length(point - points[p]) < distance) {
			return false;
		}
	}

	return true;
}

bool triangle_couvert(const Triangle &triangle, HachageSpatial &hachage_spatial, const float radius)
{
	const auto &v0 = triangle.v0;
	const auto &v1 = triangle.v1;
	const auto &v2 = triangle.v2;

	const auto centre_triangle = (v0 + v1 + v2) / 3.0f;
	const auto points = hachage_spatial.particules(centre_triangle);

	for (auto p = 0ul; p < points.size(); ++p) {
		if (glm::length(v0 - points[p]) <= radius
			&& glm::length(v1 - points[p]) <= radius
			&& glm::length(v2 - points[p]) <= radius)
		{
			return true;
		}
	}

	return false;
}

class OperateurTirageFleche : public Operateur {
public:
	OperateurTirageFleche(Noeud *noeud, const Context &contexte)
		: Operateur(noeud, contexte)
	{
		entrees(1);
		sorties(1);

		/* graine */
		add_prop("graine", "Graine", property_type::prop_int);
		set_prop_min_max(1, 100000);
		set_prop_default_value_int(1);

		/* distance */
		add_prop("distance", "Distance", property_type::prop_float);
		set_prop_min_max(0.0f, 1.0f);
		set_prop_default_value_float(0.1f);
		set_prop_tooltip("La distance minimum entre deux points.");
	}

	const char *nom_entree(size_t /*index*/) override
	{
		return "Entrée";
	}

	const char *nom_sortie(size_t /*index*/) override
	{
		return "Sortie";
	}

	const char *nom() override
	{
		return NOM_OPERATEUR;
	}

	void execute(const Context &contexte, double temps) override
	{
		entree(0)->requiers_collection(m_collection, contexte, temps);

		const auto iterateur_maillage = primitive_iterator(m_collection, Mesh::id);
		const auto maillage_entree = static_cast<Mesh *>(iterateur_maillage.get());

		if (maillage_entree == nullptr) {
			this->ajoute_avertissement("Il n'y pas de maillage dans la collection d'entrée !");
			return;
		}

		/* Convertis le maillage en triangles. */
		const auto points = maillage_entree->points();
		const auto polygones = maillage_entree->polys();

		auto aire_minimum = std::numeric_limits<float>::max();
		auto aire_maximum = 0.0f;
		auto aire_totale = 0.0f;

		/* Calcule les informations sur les aires. */
		for (auto i = 0ul; i < polygones->size(); ++i) {
			const auto polygone = (*polygones)[i];

			auto aire = calcule_aire(
					(*points)[polygone[0]],
					(*points)[polygone[1]],
					(*points)[polygone[2]]);

			aire_minimum = std::min(aire_minimum, aire);
			aire_maximum = std::max(aire_maximum, aire);
			aire_totale += aire;

			if (polygone[3] != INVALID_INDEX) {
				aire = calcule_aire(
						(*points)[polygone[0]],
						(*points)[polygone[2]],
						(*points)[polygone[3]]);

				aire_minimum = std::min(aire_minimum, aire);
				aire_maximum = std::max(aire_maximum, aire);
				aire_totale += aire;
			}
		}

		/* Place les triangles dans les boites. */
		BoiteTriangle boites[NOMBRE_BOITE];

		for (auto i = 0ul; i < polygones->size(); ++i) {
			const auto polygone = (*polygones)[i];

			auto aire = calcule_aire(
					(*points)[polygone[0]],
					(*points)[polygone[1]],
					(*points)[polygone[2]]);

			const auto index_boite = static_cast<int>(std::log2(aire_maximum / aire));

			if (index_boite < 0 || index_boite >= 64) {
				std::stringstream ss;
				ss << "Erreur lors de la génération de l'index d'une boîte !";
				ss << "\n   Index : " << index_boite;
				ss << "\n   Aire triangle : " << aire;
				ss << "\n   Aire totale : " << aire_maximum;
				this->ajoute_avertissement(ss.str());
				continue;
			}

			ajoute_triangle_boite(&boites[index_boite], (*points)[polygone[0]], (*points)[polygone[1]], (*points)[polygone[2]]);

			if (polygone[3] != INVALID_INDEX) {
				aire = calcule_aire(
						(*points)[polygone[0]],
						(*points)[polygone[2]],
						(*points)[polygone[3]]);

				const auto index_boite = static_cast<int>(std::log2(aire_maximum / aire));

				if (index_boite < 0 || index_boite >= 64) {
					std::stringstream ss;
					ss << "Erreur lors de la génération de l'index d'une boîte !";
					ss << "\n   Index : " << index_boite;
					ss << "\n   Aire triangle : " << aire;
					ss << "\n   Aire totale : " << aire_maximum;
					this->ajoute_avertissement(ss.str());
					continue;
				}

				ajoute_triangle_boite(&boites[index_boite], (*points)[polygone[0]], (*points)[polygone[2]], (*points)[polygone[3]]);
			}
		}

		/* Ne considère que les triangles dont l'aire est supérieure à ce seuil. */
		const auto seuil_aire = aire_minimum / 10000.0f;
		const auto distance = eval_float("distance");

		/* Calcule le nombre maximum de point. */
		const auto aire_cercle = M_PI * (distance * 0.5f) * (distance * 0.5f);
		const auto nombre_points = (aire_totale * DENSITE_CERCLE) / aire_cercle;
		std::cerr << "Nombre points prédits : " << nombre_points << '\n';

		auto nuage_points = static_cast<PrimPoints *>(m_collection->build("PrimPoints"));
		auto points_nuage = nuage_points->points();
		points_nuage->reserve(nombre_points);

		const auto graine = eval_int("graine");
		std::mt19937 rng(19937 + graine);
		std::uniform_real_distribution<float> dist(0.0f, 1.0f);

		HachageSpatial hachage_spatial;

		/* Tant qu'il reste des triangles à remplir... */
		while (true) {
			/* Choisis une boîte avec une probabilité proportionnelle à l'aire
			 * total des fragments de la boîte. À FAIRE. */
			BoiteTriangle *boite;
			auto boite_trouvee = false;
			auto index_boite = 0;

			for (auto i = 0; i < NOMBRE_BOITE; ++i) {
				if (boites[i].triangles.vide()) {
					continue;
				}

				boite = &boites[i];
				boite_trouvee = true;
				index_boite = i;
				break;

//				const auto probabilite_boite = boite->aire_totale / aire_totale;

//				if (dist(rng) <= probabilite_boite) {
//					boite_trouvee = true;
//					break;
//				}
			}

			/* Toutes les boites sont vides, arrêt de l'algorithme. */
			if (!boite_trouvee) {
				break;
			}

			/* Sélectionne un triangle proportionellement à son aire. */
			Triangle *triangle = boite->triangles.premier_triangle();
//			bool triangle_trouve = false;

//			for (auto tri : boite->triangles.triangles()) {
//				if (tri->jete) {
//					continue;
//				}

//				triangle = tri;
//				triangle_trouve = true;
//				break;

////				const auto probabilite_triangle = tri.aire / boite->aire_maximum;

////				if (dist(rng) <= probabilite_triangle) {
////					triangle = &tri;
////					triangle_trouve = true;
////					break;
////				}
//			}

//			if (!triangle_trouve) {
//				std::cerr << "Ne trouve pas de triangles !\n";
//				std::cerr << "Boîte : " << index_boite << '\n';
//				std::cerr << "Taille : " << boite->triangles.taille() << '\n';
//				continue;
//			}

			/* Choisis un point aléatoire p sur le triangle en prenant une
			 * coordonnée barycentrique aléatoire. */
			const auto v0 = triangle->v0;
			const auto v1 = triangle->v1;
			const auto v2 = triangle->v2;
			const auto e0 = v1 - v0;
			const auto e1 = v2 - v0;

			auto r = dist(rng);
			auto s = dist(rng);

			if (r + s >= 1.0f) {
				r = 1.0f - r;
				s = 1.0f - s;
			}

			auto point = v0 + r * e0 + s * e1;

			/* Vérifie que le point respecte la condition de distance minimal */
			auto ok = verifie_distance_minimal(hachage_spatial, point, distance);

			if (ok) {
				hachage_spatial.ajoute(point);
				points_nuage->push_back(point);
			}

			/* Vérifie si le triangle est complétement couvert par un point de
			 * l'ensemble. */
			auto couvert = triangle_couvert(*triangle, hachage_spatial, distance);

			if (couvert) {
				/* Si couvert, jète le triangle. */
				boite->aire_totale -= triangle->aire;
				boite->triangles.enleve(triangle);
			}
			else {
				/* Sinon, coupe le triangle en petit morceaux, et ajoute ceux
				 * qui ne ne sont pas totalement couvert à la liste, sauf si son
				 * aire est plus petite que le seuil d'acceptance. */

				/* On coupe le triangle en quatre en introduisant un point au
				 * centre de chaque coté. */
				const auto v01 = (v0 + v1) * 0.5f;
				const auto v12 = (v1 + v2) * 0.5f;
				const auto v20 = (v2 + v0) * 0.5f;

				Triangle triangle_fils[4] = {
					Triangle{ v0, v01, v20},
					Triangle{v01,  v1, v12},
					Triangle{v12,  v2, v20},
					Triangle{v20, v01, v12},
				};

				for (auto i = 0; i < 4; ++i) {

					const auto aire = calcule_aire(triangle_fils[i]);

					if (std::abs(aire - seuil_aire) <= std::numeric_limits<float>::epsilon()) {
						boite->aire_totale -= aire;
						continue;
					}

					couvert = triangle_couvert(triangle_fils[i], hachage_spatial, distance);

					if (couvert) {
						continue;
					}

					const auto index_boite = static_cast<int>(std::log2(aire_maximum / aire));

					if (index_boite >= 0 && index_boite < 64) {
						auto &b = boites[index_boite];

						b.triangles.ajoute(triangle_fils[i].v0, triangle_fils[i].v1, triangle_fils[i].v2);
						b.aire_totale += aire;
					}
				}

				boite->triangles.enleve(triangle);
			}
		}

		std::cerr << "Nombre de points : " << points_nuage->size() << "\n";
		std::cerr << "Nombre d'alvéoles : " << hachage_spatial.taille() << '\n';
	}
};

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<OperateurTirageFleche>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "Géométrie"));
}

}
