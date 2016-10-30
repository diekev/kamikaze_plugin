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
#include <kamikaze/nodes.h>

#include <fstream>
#include <sstream>

static constexpr auto NODE_NAME = "OBJ Reader";

struct MeshInfo {
	size_t num_verts = 0;
	size_t num_normals = 0;
	size_t num_uvs = 0;
	size_t num_polys = 0;
};

class NodeMeshOBJRead : public Node {
	std::vector<MeshInfo *> m_meshes_info;
	size_t m_num_verts = 0;
	size_t m_num_normals = 0;
	size_t m_num_uvs = 0;
	size_t m_num_polys = 0;

public:
	NodeMeshOBJRead();

	bool prereadObj(const std::string &filename);

	void process() override;
	void createMesh(PointList *points, int mesh_index, PolygonList *polys, Attribute *normals, MeshInfo *info, Mesh *mesh);
};

NodeMeshOBJRead::NodeMeshOBJRead()
    : Node(NODE_NAME)
{
	addOutput("Mesh");

	add_prop("filename", "Filename", property_type::prop_input_file);
}

void NodeMeshOBJRead::process()
{
	const auto filename = eval_string("filename");

	if (filename.empty() || !prereadObj(filename)) {
		return;
	}

	if (!prereadObj(filename)) {
		return;
	}

	MeshInfo *info;
	Mesh *mesh = nullptr;
	PointList *points;
	PolygonList *polys;
	Attribute *normals;
	int normal_idx = 0;

	std::ifstream fp_in;
	fp_in.open(filename.c_str());

	glm::vec3 v;

	std::string line;
	std::string header;
	std::string faceinfo;

	int poly[4];
	int uv[4];
	int normal[4];

	bool is_new_mesh = true;
	int mesh_index = 0;

	/* TODOs:
	 * - handle cases where there are multiple objects defined in a file
	 * - material handling
	 */

	while (fp_in.good()) {
		std::getline(fp_in, line);

		if (line.empty() || line[0] == '#') {
			continue;
		}

		std::istringstream is(line);
		is >> header;

		if (header == "v") {
			if (is_new_mesh) {
				info = m_meshes_info[mesh_index++];

				std::cerr << "Creating mesh: \n";
				std::cerr << "Num verts: " << info->num_verts << '\n';
				std::cerr << "Num polys: " << info->num_polys << '\n';
				std::cerr << "Num normals: " << info->num_normals << '\n';

				mesh = static_cast<Mesh *>(m_collection->build("Mesh"));

				points = mesh->points();
				points->reserve(info->num_verts);

				polys = mesh->polys();
				polys->reserve(info->num_polys);

				normals = mesh->attribute("normal", ATTR_TYPE_VEC3);
				normals->resize(info->num_normals);

				normal_idx = 0;

				is_new_mesh = false;
			}

			is >> v.x >> v.y >> v.z;
			points->push_back(v);
		}
		else if (header == "vn") {
			is >> v.x >> v.y >> v.z;
			normals->vec3(normal_idx++, v);
		}
		else if (header == "f") {
			int i = 0;

			while (is >> faceinfo) {
				auto s1 = faceinfo.find_first_of('/', 0);
				auto s2 = faceinfo.find_first_of('/', s1 + 1);

				poly[i] = std::stoi(faceinfo.substr(0, s1)) - 1;

				if (info->num_uvs > 0) {
					uv[i] = std::stoi(faceinfo.substr(s1 + 1, (s2 - s1) - 1)) - 1;
				}

				if (info->num_normals > 0) {
					normal[i] = std::stoi(faceinfo.substr(s2 + 1)) - 1;
				}

				++i;
			}

			if (i == 3) {
				poly[3] = INVALID_INDEX;
			}

			polys->push_back(glm::uvec4{ poly[0], poly[1], poly[2], poly[3] });
		}
		else if (header == "o") {
			if (is_new_mesh) {

				info = m_meshes_info[mesh_index++];
				std::cerr << "Creating mesh: \n";
				std::cerr << "Num verts: " << info->num_verts << '\n';
				std::cerr << "Num polys: " << info->num_polys << '\n';
				std::cerr << "Num normals: " << info->num_normals << '\n';

				mesh = static_cast<Mesh *>(m_collection->build("Mesh"));

				points = mesh->points();
				points->reserve(info->num_verts);

				polys = mesh->polys();
				polys->reserve(info->num_polys);

				normals = mesh->attribute("normal", ATTR_TYPE_VEC3);
				normals->resize(info->num_normals);

				normal_idx = 0;

				is_new_mesh = false;
			}

			is >> faceinfo;
			mesh->name(faceinfo.c_str());
		}
		else if (header == "g") {
			if (is_new_mesh) {

				info = m_meshes_info[mesh_index++];
				std::cerr << "Creating mesh: \n";
				std::cerr << "Num verts: " << info->num_verts << '\n';
				std::cerr << "Num polys: " << info->num_polys << '\n';
				std::cerr << "Num normals: " << info->num_normals << '\n';

				mesh = static_cast<Mesh *>(m_collection->build("Mesh"));

				points = mesh->points();
				points->reserve(info->num_verts);

				polys = mesh->polys();
				polys->reserve(info->num_polys);

				normals = mesh->attribute("normal", ATTR_TYPE_VEC3);
				normals->resize(info->num_normals);

				normal_idx = 0;

				is_new_mesh = false;
			}
			else {
				is_new_mesh = true;
			}

			is >> faceinfo;
			mesh->name(faceinfo.c_str());
		}
	}

	fp_in.close();

	mesh->tagUpdate();
}

bool NodeMeshOBJRead::prereadObj(const std::string &filename)
{
	for (auto &mesh_info : m_meshes_info) {
		delete mesh_info;
	}

	m_meshes_info.clear();

	std::ifstream fp_in;
	fp_in.open(filename.c_str());

	if (!fp_in.is_open()) {
		return false;
	}

	MeshInfo *info;
	bool is_new_mesh = true;

	while (fp_in.good()) {
		std::string line;
		std::getline(fp_in, line);

		if (line.empty()) {
			continue;
		}

		const auto &header = line.substr(0, 2);

		if (header.compare("v ") == 0) {
			if (is_new_mesh) {
				info = new MeshInfo;
				m_meshes_info.push_back(info);
				is_new_mesh = false;
			}

			info->num_verts++;
		}
		else if (header.compare("vt") == 0) {
//			info->num_uvs++;
		}
		else if (header.compare("vn") == 0) {
			info->num_normals++;
		}
		else if (header.compare("f ") == 0) {
			info->num_polys++;
		}
		else if (header.compare("g ") == 0) {
			is_new_mesh = true;
		}
	}

	fp_in.close();
	return true;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("Mesh", NODE_NAME, NodeMeshOBJRead);
}

}
