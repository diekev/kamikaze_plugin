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

/**
 * @file node_alpha_mesh.cc
 * @author Kévin Dietrich
 *
 * Implementation of the alpha mesh algorithm from
 * "Enhancing Particle Methods for Fluid Simulation in Computer Graphics",
 * Hagit Schechter, 2013
 */

#include <kamikaze/mesh.h>
#include <kamikaze/operateur.h>
#include <kamikaze/prim_points.h>

static constexpr auto NOM_OPERATEUR = "Alpha Mesh From Points";
static constexpr auto AIDE_OPERATEUR = "";

class NodeAlphaMesh : public Operateur {
	float m_radius = 1.0f;

public:
	NodeAlphaMesh(Noeud *noeud, const Context &contexte);

	const char *nom_entree(size_t /*index*/) override { return ""; }
	const char *nom_sortie(size_t /*index*/) override { return ""; }

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override;
};

NodeAlphaMesh::NodeAlphaMesh(Noeud *noeud, const Context &contexte)
	: Operateur(noeud, contexte)
{
	entrees(1);
	sorties(1);

	add_prop("point_radius", "Point Radius", property_type::prop_float);
	set_prop_min_max(0.0f, 2.0f);
	set_prop_default_value_float(1.0f);
	set_prop_tooltip("Set the uniform radius of the points.");
}

bool build_sphere(const glm::vec3 &x0,
                  const glm::vec3 &x1,
                  const glm::vec3 &x2,
                  const float radius,
                  glm::vec3 &center)
{
	const glm::vec3 &x0x1 = x0 - x1;
	const float lx0x1 = glm::length(x0x1);

	const glm::vec3 &x1x2 = x1 - x2;
	const float lx1x2 = glm::length(x1x2);

	const glm::vec3 &x2x0 = x2 - x0;
	const float lx2x0 = glm::length(x2x0);

	const glm::vec3 n = glm::cross(x0x1, x1x2);
	const float len_n = glm::length(n);

	const float radius_x = (lx0x1 * lx1x2 * lx2x0) / (2.0f * len_n);

	if (radius_x > radius) {
		return false;
	}

	const float lx0x2 = glm::length(x1 - x2);
	const float abs_n_sqr = (len_n * len_n);
	const float inv_abs_n_sqr = 1.0f / abs_n_sqr;
	const float inv_abs_n_sqr2 = 1.0f / (2.0f * abs_n_sqr);

	const float alpha = (lx1x2 * lx1x2 * glm::dot(x0x1, x0 - x2)) * inv_abs_n_sqr2;
	const float beta  = (lx0x2 * lx0x2 * glm::dot(x1 - x0, x1x2)) * inv_abs_n_sqr2;
	const float gamma = (lx0x1 * lx0x1 * glm::dot(x2x0, x2 - x1)) * inv_abs_n_sqr2;

	const glm::vec3 l = alpha * x0 + beta * x1 + gamma * x2;
	const float t = sqrtf((radius_x * radius_x - radius * radius) * inv_abs_n_sqr);

	center = l + t * n;

	if (glm::any(glm::isnan(center))) {
		return false;
	}

	return true;
}

/**
 * @brief find_neighbour_points Find points in the neighbourhood of another
 *                              point within a given radius.
 *
 * @param points  The points to traverse.
 * @param rpoints The returned set of points.
 * @param point   The reference point.
 * @param radius  The search radius.
 */
void find_neighbour_points(const PointList &points,
                           PointList &rpoints,
                           const glm::vec3 &point,
                           const float radius)
{
	for (size_t i = 0; i < points.size(); ++i) {
		const auto &pi = points[i];

		if (pi == point) {
			continue;
		}

		if (glm::length(point - pi) < radius) {
			rpoints.push_back(pi);
		}
	}
}

void get_poly(PointList *mpoints,
              int &tri_offset,
              const float radius,
              const PointList &N1,
              const glm::vec3 &pi,
              PolygonList *mpolys)
{
	for (size_t j(0); j < N1.size() - 1; ++j) {
		auto pj = N1[j], pk = N1[j + 1];
		glm::vec3 center;

		if (!build_sphere(pi, pj, pk, radius, center)) {
			continue;
		}

		bool clear = true;

		for (size_t k(0); k < N1.size(); ++k) {
			if (k == j || k == (j + 1)) {
				continue;
			}

			if (glm::length(N1[k] - center) < radius) {
				clear = false;
				break;
			}
		}

		if (!clear) {
			continue;
		}

		mpoints->push_back(pi);
		mpoints->push_back(pj);
		mpoints->push_back(pk);

		mpolys->push_back(glm::uvec4(tri_offset + 0,
		                             tri_offset + 1,
		                             tri_offset + 2,
		                             std::numeric_limits<unsigned int>::max()));

		tri_offset += 3;
	}
}

void construct_alpha_mesh(PointList &points, const float radius,
                          Mesh *mesh)
{
	auto tri_offset = 0;
	auto mpoints = mesh->points();
	auto mpolys = mesh->polys();

	for (size_t i = 0; i < points.size(); ++i) {
		auto pi = points[i];

		PointList N1;
		find_neighbour_points(points, N1, pi, 2.0f * radius);

		get_poly(mpoints, tri_offset, radius, N1, pi, mpolys);
	}
}

void NodeAlphaMesh::execute(const Context &contexte, double temps)
{
	entree(0)->requiers_collection(m_collection, contexte, temps);

	const auto radius = eval_float("point_radius");

	for (auto &prim : primitive_iterator(m_collection, PrimPoints::id)) {
		auto points = static_cast<PrimPoints *>(prim);
		auto mesh = static_cast<Mesh *>(m_collection->build("Mesh"));

		construct_alpha_mesh(*(points->points()), radius, mesh);

		std::cerr << *mesh << '\n';
		mesh->tagUpdate();
	}
}

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<NodeAlphaMesh>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "Mesh"));
}

}

#if 0
struct Triangle {
	glm::vec3 x0, x1, x2;
	bool processed;
};

typedef std::vector<Particle *> ParticleList;


struct alpha_face {
	int t1, t2, t3, t4, t5, t6;
};

void build_apha_graph(const std::vector<glm::vec3> &aplha_vertices,
                      const std::vector<Triangle> &alpha_triangles,
                      std::vector<alpha_face> &faces)
{
	for (size_t i = 0; i < aplha_vertices.size(); ++i) {
		// set of triangles which include aplha_vertices[i]
		std::vector<int> Tp(6);
		glm::vec3 p = alpha_vertices[i];

		for (size_t j(0); j < alpha_triangles.size(); ++j) {
			Triangle t = alpha_triangles[j];

			if (t.x0 == p || t.x1 == p || t.x2 == p) {
				t.processed = false;
				Tp.push_back(j);
			}
		}

		for (size_t j(0); j < Tp.size(); ++j) {
			Triangle ti = alpha_triangles[Tp[j]];

			if (ti.processed) {
				continue;
			}

			std::vector<Triangle> Tpi(Tp.size() - (j + 1));
			for (size_t k(j + 1); k < Tp.size(); ++k) {
				Tpi.push_back(Tp[k]);
			}

			// TODO: sort Tp by signed volume w.r.t. ti

			Triangle tcurr = Tpi[0], tfirst = Tpi[0];

			while (true) {
				tcurr.processed = true;
				// TODO

				if (1) {
					break;
				}
			}
		}
	}
}

void smooth_beta_mesh(std::vector<glm::vec3> &beta_vertices,
                      std::vector<Triangle> &beta_triangles,
                      std::vector<glm::ivec4> &beta_faces)
{
	std::vector<glm::vec3> smooth_verts(beta_vertices.size());

	for (size_t i = 0; i < beta_vertices.size(); ++i) {
		std::vector<glm::ivec4> vert_faces; // faces containing beta_vertices[i]
		glm::vec3 vnew(0.0f);
		float weight(0.0f);

		for (size_t f(0); f < vert_faces.size(); ++f) {
			// for each edge in face
			glm::ivec4 face = vert_faces[f];
			{
				glm::vec3 pa = beta_vertices[face[0]];
				glm::vec3 pb = beta_vertices[face[1]];
				float w = glm::length(pa, pb);
				glm::vec3 pp = (pa + pb) * 0.5f;
				weight += w;
				vnew += pp * w;
			}
		}

		vnew /= weight;
		smooth_verts.push_back(vnew);
	}

	std::copy(smooth_verts.begin(), smooth_verts.end(), beta_vertices.begin());

	std::vector<glm::vec3> ab_verts;

	for (size_t i = 0; i < ab_verts.size(); ++i) {
		glm::ivec4 face; // face of ab_verts[i];
		glm::vec3 qnew(0.0f);
		float weight(0.0f);

		// for each edge in face
		{
			glm::vec3 qa = beta_vertices[face[0]];
			glm::vec3 qb = beta_vertices[face[1]];
			float w = glm::length(qa, qb);
			glm::vec3 qp = (qa + qb) * 0.5f;
			weight += w;
			qnew += qp * w;
		}

		ab_verts[i] = qnew / weight;
	}
}
#endif
