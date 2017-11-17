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

struct int3 {
	size_t x, y, z;

	bool operator<(const int3 &autre) const
	{
		if (x < autre.x) {
			return true;
		}

		if (x == autre.x && y < autre.y) {
			return true;
		}

		if (x == autre.x && y == autre.y && z < autre.z) {
			return true;
		}

		return false;
	}
};

#define HASH_SPATIAL

struct HashSpatial {
	std::map<int3, std::vector<glm::vec3>> m_tableau;
	float m_taille = 1.0f;
	float m_taille_inverse = 1.0f;
	size_t m_nombre_cellule;

	int3 fonction_hash(const glm::vec3 &particle)
	{
		int3 ret;

		ret.x = static_cast<size_t>(particle.x * m_taille_inverse);
		ret.y = static_cast<size_t>(particle.y * m_taille_inverse);
		ret.z = static_cast<size_t>(particle.z * m_taille_inverse);

		return ret;
	}

	void ajoute(const glm::vec3 &particle)
	{
		auto hash = fonction_hash(particle);
		m_tableau[hash].push_back(particle);
	}

	const std::vector<glm::vec3> &particules(const glm::vec3 &particule)
	{
		auto hash = fonction_hash(particule);
		return m_tableau[hash];
	}

	size_t taille() const
	{
		return m_tableau.size();
	}
};

/* ************************************************************************** */

struct Triangle {
	glm::vec3 v0 = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 v1 = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 v2 = glm::vec3(0.0f, 0.0f, 0.0f);
	float aire = 0.0f;
	bool jete = false;
	size_t index = 0;
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

float calcule_aire(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2)
{
	const auto c1 = v1 - v0;
	const auto c2 = v2 - v0;

	return glm::length(glm::cross(c1, c2)) * 0.5f;
}

float calcule_aire(const Triangle &triangle)
{
	const auto c1 = triangle.v1 - triangle.v0;
	const auto c2 = triangle.v2 - triangle.v0;

	return glm::length(glm::cross(c1, c2)) * 0.5f;
}

class ListeTriangle {
	std::vector<Triangle *> m_triangles{};
	std::stack<int, std::vector<int>> m_pile_index{};

	int m_nombre_triangles = 0;

	Triangle *m_premier_triangle = nullptr;
	Triangle *m_dernier_triangle = nullptr;

public:
	ListeTriangle() = default;

	~ListeTriangle()
	{
		for (Triangle *triangle : m_triangles) {
			delete triangle;
		}
	}

	Triangle *ajoute(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2)
	{
		Triangle *triangle;

		if (m_pile_index.empty()) {
			triangle = new Triangle(v0, v1, v2);
			triangle->index = m_triangles.size();
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

			//m_triangles.push_back(triangle);
		}
		else {
			auto index = m_pile_index.top();
			m_pile_index.pop();

			triangle = m_triangles[index];
			triangle->index = index;
			triangle->v0 = v0;
			triangle->v1 = v1;
			triangle->v2 = v2;
			triangle->aire = calcule_aire(*triangle);
			triangle->jete = false;
		}

		++m_nombre_triangles;

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
//		auto index = triangle->index;
//		auto size = m_triangles.size();

//		if (m_triangles[index] != m_triangles[size - 1]){
//			assert(triangle == m_triangles[index]);

//			m_triangles[index] = m_triangles[size - 1];
//			m_triangles[m_nombre_triangles - 1] = triangle;
//			m_triangles[index]->index = index;

//			assert(triangle != m_triangles[index]);
//		}

//		m_triangles.pop_back();
		delete triangle;

//		triangle->jete = true;
//		m_pile_index.push(triangle->index);
//		--m_nombre_triangles;

	}

	Triangle *premier_triangle()
	{
		return m_premier_triangle;
	}

	size_t taille() const
	{
		return m_nombre_triangles;
	}

	std::vector<Triangle *> &triangles()
	{
		return m_triangles;
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

#ifdef HASH_SPATIAL
bool verifie_distance_minimal(HashSpatial &hash, const glm::vec3 &point, float distance)
{
	const auto points = hash.particules(point);

	for (auto p = 0ul; p < points.size(); ++p) {
		if (glm::length(point - points[p]) < distance) {
			return false;
		}
	}

	return true;
}

bool triangle_couvert(const Triangle &triangle, HashSpatial &hash, const float radius)
{
	const auto &v0 = triangle.v0;
	const auto &v1 = triangle.v1;
	const auto &v2 = triangle.v2;

	const auto centre_triangle = (v0 + v1 + v2) / 3.0f;
	const auto points = hash.particules(centre_triangle);

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
#else
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
#endif

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
//		auto nombre_triangles = 0ul;

//		for (auto i = 0ul; i < polygones->size(); ++i) {
//			const auto polygone = (*polygones)[i];

//			nombre_triangles += ((polygone[3] == INVALID_INDEX) ? 1 : 2);
//		}

//		std::vector<Triangle> triangles;
//		triangles.reserve(nombre_triangles);

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

		if (aire_minimum <= 0.0f) {
			std::stringstream ss;
			ss << "Erreur : l'aire minimale est inférieure ou égale à 0 !";
			ss << "\n   Aire minimale : " << aire_minimum;
			this->ajoute_avertissement(ss.str());
			return;
		}

		if (aire_maximum <= 0.0f) {
			std::stringstream ss;
			ss << "Erreur : l'aire maximale est inférieure ou égale à 0 !";
			ss << "\n   Aire maximale : " << aire_maximum;
			this->ajoute_avertissement(ss.str());
			return;
		}

		if (aire_maximum < aire_minimum) {
			std::stringstream ss;
			ss << "Erreur : l'aire maximale est inférieure à l'aire minimale !";
			ss << "\n   Aire minimale : " << aire_minimum;
			ss << "\n   Aire maximale : " << aire_maximum;
			this->ajoute_avertissement(ss.str());
			return;
		}

		/* Place les triangles dans les boites. */
		BoiteTriangle boites[NOMBRE_BOITE];

//		for (auto i = 0ul; i < nombre_triangles; ++i) {
//			const auto &triangle = triangles[i];

//			const auto index_boite = static_cast<int>(std::log2(aire_maximum / std::abs(triangle.aire)));

//			if (index_boite < 0 || index_boite >= 64) {
//				std::stringstream ss;
//				ss << "Erreur lors de la génération de l'index d'une boîte !";
//				ss << "\n   Index : " << index_boite;
//				ss << "\n   Aire triangle : " << triangle.aire;
//				ss << "\n   Aire totale : " << aire_maximum;
//				this->ajoute_avertissement(ss.str());
//				continue;
//			}

//			auto &boite = boites[index_boite];

//			boite.triangles.ajoute(triangle.v0, triangle.v1, triangle.v2);
//			boite.aire_minimum = std::min(boite.aire_minimum, triangle.aire);
//			boite.aire_maximum = 2 * boite.aire_minimum;
//			boite.aire_totale += triangle.aire;
//		}

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

#ifdef HASH_SPATIAL
		HashSpatial hash;
#endif

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
#ifdef HASH_SPATIAL
			auto ok = verifie_distance_minimal(hash, point, distance);
#else
			auto ok = verifie_distance_minimal(*points_nuage, point, distance);
#endif

			if (ok) {
#ifdef HASH_SPATIAL
				hash.ajoute(point);
#endif
				points_nuage->push_back(point);
			}

			/* Vérifie si le triangle est complétement couvert par un point de
			 * l'ensemble. */
#ifdef HASH_SPATIAL
			auto couvert = triangle_couvert(*triangle, hash, distance);
#else
			auto couvert = triangle_couvert(*triangle, *points_nuage, distance);
#endif

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

#ifdef HASH_SPATIAL
					couvert = triangle_couvert(triangle_fils[i], hash, distance);
#else
					couvert = triangle_couvert(triangle_fils[i], *points_nuage, distance);
#endif

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
		std::cerr << "Taille hash : " << hash.taille() << '\n';
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
