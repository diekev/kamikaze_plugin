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

#include <random>
#include <sstream>
#include <stack>

/**
 * "Dart Throwing on Surfaces" (Cline et al. 2009)
 * http://peterwonka.net/Publications/pdfs/2009.EGSR.Cline.PoissonSamplingOnSurfaces.pdf
 */

static constexpr auto NOMBRE_BOITE = 64;

static const char *NOM_OPERATEUR = "Tirage de flèche";
static const char *AIDE_OPERATEUR =
		"Crée des points sur une surface en l'échantillonnant par tirage de flèche.";

struct Triangle {
	glm::vec3 v0{0.0f, 0.0f, 0.0f}, v1{0.0f, 0.0f, 0.0f}, v2{0.0f, 0.0f, 0.0f};
	float aire = 0.0f;
	bool jete = false;
	int index = 0;

	Triangle() = default;

	Triangle(const glm::vec3 &v_0, const glm::vec3 &v_1, const glm::vec3 &v_2)
		: Triangle()
	{
		v0 = v_0;
		v1 = v_1;
		v2 = v_2;
	}

	Triangle(const Triangle &triangle) = default;
};

float calcule_aire(const Triangle &triangle)
{
	const auto c1 = triangle.v1 - triangle.v0;
	const auto c2 = triangle.v2 - triangle.v0;

	return glm::abs(glm::length(glm::cross(c1, c2)) * 0.5f);
}

class ListeTriangle {
	std::vector<Triangle> m_triangles{};
	std::stack<int, std::vector<int>> m_pile_index{};

	int m_nombre_triangles = 0;

public:
	ListeTriangle() = default;

	void ajoute(Triangle triangle)
	{
		std::cerr << "Début : " << __func__ << '\n';

		if (m_pile_index.empty()) {
			triangle.index = m_triangles.size();
			m_triangles.push_back(triangle);
		}
		else {
			auto index = m_pile_index.top();
			m_pile_index.pop();

			triangle.index = index;
			m_triangles[index] = triangle;
		}

		++m_nombre_triangles;
		std::cerr << "Fin : " << __func__ << '\n';
	}

	void enleve(Triangle &triangle)
	{
		triangle.jete = true;
		m_pile_index.push(triangle.index);
		--m_nombre_triangles;
	}

	std::vector<Triangle> &triangles()
	{
		return m_triangles;
	}

	bool vide() const
	{
		return m_nombre_triangles == 0;
	}
};

struct BoiteTriangle {
	float aire_minimum = std::numeric_limits<float>::max();
	float aire_maximum = 0.0f; // = 2 * aire_minimum
	float aire_totale = 0.0f;

	ListeTriangle triangles;
};

/* À FAIRE : à optimiser avec une table de hachage grille spatiale ou un arbre-KD. */
bool verifie_distance_minimal(const PointList &points, const glm::vec3 &point, float distance)
{
	for (auto p = 0ul; p < points.size(); ++p) {
		if (glm::length(point - points[p]) < distance) {
			return false;
		}
	}

	return true;
}

/* À FAIRE : à optimiser avec une table de hachage grille spatiale ou un arbre-KD. */
bool triangle_couvert(const Triangle &triangle, const PointList &points, const float radius)
{
	for (auto p = 0ul; p < points.size(); ++p) {
		const auto &v0 = triangle.v0;
		const auto &v1 = triangle.v1;
		const auto &v2 = triangle.v2;

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
		auto nombre_triangles = 0ul;

		for (auto i = 0ul; i < polygones->size(); ++i) {
			const auto polygone = (*polygones)[i];

			nombre_triangles += ((polygone[3] == INVALID_INDEX) ? 1 : 2);
		}

		auto aire_minimum = std::numeric_limits<float>::max();
		auto aire_maximum = 0.0f;
		auto aire_totale = 0.0f;

		std::vector<Triangle> triangles;
		triangles.reserve(nombre_triangles);

		for (auto i = 0ul; i < polygones->size(); ++i) {
			const auto polygone = (*polygones)[i];

			Triangle triangle;
			triangle.v0 = (*points)[polygone[0]];
			triangle.v1 = (*points)[polygone[1]];
			triangle.v2 = (*points)[polygone[2]];
			triangle.aire = calcule_aire(triangle);

			triangles.push_back(triangle);

			aire_minimum = std::min(aire_minimum, triangle.aire);
			aire_maximum = std::max(aire_maximum, triangle.aire);
			aire_totale += triangle.aire;

			if (polygone[3] != INVALID_INDEX) {
				Triangle triangle2;
				triangle2.v0 = (*points)[polygone[0]];
				triangle2.v1 = (*points)[polygone[2]];
				triangle2.v2 = (*points)[polygone[3]];
				triangle2.aire = calcule_aire(triangle);

				triangles.push_back(triangle2);

				aire_minimum = std::min(aire_minimum, triangle2.aire);
				aire_maximum = std::max(aire_maximum, triangle2.aire);
				aire_totale += triangle2.aire;
			}
		}

		std::cerr << "Placement des triangles dans les boîtes....\n";

		/* Place les triangles dans les boites. */
		BoiteTriangle boites[NOMBRE_BOITE];

		for (auto i = 0ul; i < nombre_triangles; ++i) {
			const auto &triangle = triangles[i];

			const auto index_boite = static_cast<int>(std::log2(aire_maximum / std::abs(triangle.aire)));

			if (index_boite < 0 || index_boite >= 64) {
				std::stringstream ss;
				ss << "Erreur lors de la génération de l'index d'une boîte !";
				ss << "\n   Index : " << index_boite;
				ss << "\n   Aire triangle : " << triangle.aire;
				ss << "\n   Aire totale : " << aire_maximum;
				this->ajoute_avertissement(ss.str());
				break;
			}

			auto &boite = boites[index_boite];

			boite.triangles.ajoute(triangle);
			boite.aire_minimum = std::min(boite.aire_minimum, triangle.aire);
			boite.aire_maximum = 2 * boite.aire_minimum;
			boite.aire_totale += triangle.aire;
		}

		std::cerr << "Création de la primitive nuage de points.\n";

		auto nuage_points = static_cast<PrimPoints *>(m_collection->build("PrimPoints"));
		auto points_nuage = nuage_points->points();

		std::mt19937 rng(19937);
		std::uniform_real_distribution<float> dist(0.0f, 1.0f);

		/* Ne considère que les triangles dont l'aire est supérieure à ce seuil. */
		const auto seuil_aire = aire_minimum / 10000.0f;
		const auto distance = 0.01f;

		std::cerr << "Lancement de l'algorithme...\n";

		/* Tant qu'il reste des triangles à remplir... */
		while (true) {
			std::cerr << "----------------------------------------------------\n";
			/* Choisis une boîte avec une probabilité proportionnelle à l'aire
			 * total des fragments de la boîte. À FAIRE. */
			BoiteTriangle *boite;
			auto boite_trouvee = false;

			std::cerr << "Tirage d'une boîte...\n";
			for (auto i = 0; i < NOMBRE_BOITE; ++i) {
				if (boites[i].triangles.vide()) {
					continue;
				}

				boite = &boites[i];
				boite_trouvee = true;
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
			Triangle *triangle;
			bool triangle_trouve = false;

			std::cerr << "Tirage d'un triangle...\n";
			for (auto &tri : boite->triangles.triangles()) {
				if (tri.jete) {
					continue;
				}
				triangle = &tri;
				triangle_trouve = true;
				break;

//				const auto probabilite_triangle = tri.aire / boite->aire_maximum;

//				if (dist(rng) <= probabilite_triangle) {
//					triangle = &tri;
//					triangle_trouve = true;
//					break;
//				}
			}

			if (!triangle_trouve) {
				continue;
			}

			/* Choisis un point aléatoire p sur le triangle en prenant une
			 * coordonnée barycentrique aléatoire. */
			std::cerr << "Coordonnées barycentrique...\n";
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
			std::cerr << "Distance minimal...\n";
			auto ok = verifie_distance_minimal(*points, point, distance);

			if (ok) {
				std::cerr << "Ajoute point au nuage...\n";
				points_nuage->push_back(point);
			}

			/* Vérifie si le triangle est complétement couvert par un point de
			 * l'ensemble. */
			std::cerr << "Triangle couvert...\n";
			auto couvert = triangle_couvert(*triangle, *points, distance);

			if (couvert) {
				std::cerr << "Suppression du triangle...\n";
				/* Si couvert, jète le triangle. */
				boite->triangles.enleve(*triangle);
				boite->aire_totale -= triangle->aire;
			}
			else {
				std::cerr << "Coupe du triangle...\n";
				std::cerr << "Coordonnées : " << v0 << ", " << v1 << ", " << v2 << '\n';

				/* Sinon, coupe le triangle en petit morceaux, et ajoute ceux
				 * qui ne ne sont pas totalement couvert à la liste, sauf si son
				 * aire est plus petite que le seuil d'acceptance. */

				/* On coupe le triangle en quatre en introduisant un point au
				 * centre de chaque coté. */
				const auto v01 = v0 + (v1 - v0) * 0.5f;
				const auto v12 = v1 + (v2 - v1) * 0.5f;
				const auto v20 = v2 + (v0 - v2) * 0.5f;

				assert(v0 != v1);
				assert(v0 != v2);
				assert(v1 != v2);

				assert(v0 != v01);
				assert(v0 != v12);
				assert(v0 != v20);

				assert(v1 != v01);
				assert(v1 != v12);
				assert(v1 != v20);

				assert(v2 != v01);
				assert(v2 != v12);
				assert(v2 != v20);

				assert(v01 != v12);
				assert(v01 != v20);
				assert(v12 != v20);

				assert((v0 != v01) && (v01 != v20) && (v0 != v20));
				assert((v01 != v1) && (v1 != v12) && (v12 != v01));
				assert((v12 != v2) && (v2 != v20) && (v20 != v12));
				assert((v20 != v01) && (v01 != v12) && (v12 != v20));

				Triangle triangle_fils[4] = {
					Triangle{ v0, v01, v20},
					Triangle{v01,  v1, v12},
					Triangle{v12,  v2, v20},
					Triangle{v20, v01, v12},
				};

				for (auto i = 0; i < 4; ++i) {
					std::cerr << "--- Triangle fils : " << i << '\n';

					const auto aire = calcule_aire(triangle_fils[i]);

					if (aire <= seuil_aire) {
						std::cerr << "    Aire inférireure au seuil : "
								  << "aire : " << aire
								  << ", seuil : " << seuil_aire << '\n';
						std::cerr << "    Coordonnées : "
								  << triangle_fils[i].v0 << ", "
								  << triangle_fils[i].v1 << ", "
								  << triangle_fils[i].v2 << '\n';
						boite->aire_totale -= aire;
						continue;
					}

					couvert = triangle_couvert(triangle_fils[i], *points, distance);

					if (couvert) {
						std::cerr << "    Triangle couvert.\n";
						continue;
					}

					const auto index_boite = static_cast<int>(std::log2(aire_maximum / aire));

					if (index_boite >= 0 && index_boite < 64) {
						std::cerr << "    Ajout du triangle dans la boîte : " << index_boite << '\n';
						auto &b = boites[index_boite];

						b.triangles.ajoute(triangle_fils[i]);
						b.aire_totale += aire;
					}
					else {
						std::cerr << "    Erreur lors de la génération de l'index d'une boîte !";
						std::cerr << "\n       Index : " << index_boite;
						std::cerr << "\n       Aire triangle : " << aire;
						std::cerr << "\n       Aire totale : " << aire_maximum;
					}
				}

				boite->triangles.enleve(*triangle);
			}
		}
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
