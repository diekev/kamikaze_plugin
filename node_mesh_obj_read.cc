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
#include <kamikaze/paramfactory.h>

#include <fstream>
#include <sstream>

static constexpr auto NODE_NAME = "OBJ Reader";

class NodeMeshOBJRead : public Node {
	std::string m_filename;
	size_t m_num_verts = 0;
	size_t m_num_normals = 0;
	size_t m_num_uvs = 0;
	size_t m_num_polys = 0;

public:
	NodeMeshOBJRead();

	bool prereadObj(const std::string &filename);

	void setUIParams(ParamCallback *cb) override;
	void process() override;
};

NodeMeshOBJRead::NodeMeshOBJRead()
    : Node(NODE_NAME)
{
	addOutput("Mesh");
}

void NodeMeshOBJRead::setUIParams(ParamCallback *cb)
{
	file_param(cb, "Filename", &m_filename);
}

void NodeMeshOBJRead::process()
{
	if (m_filename.empty() || !prereadObj(m_filename)) {
		setOutputPrimitive("Mesh", nullptr);
		return;
	}

	Mesh *mesh = new Mesh;
	mesh->verts().reserve(m_num_verts);
	mesh->quads().reserve(m_num_polys);
	mesh->tris().reserve(0);
	mesh->normals().reserve(m_num_normals);

	std::ifstream fp_in;
	fp_in.open(m_filename.c_str());

	glm::vec3 v;

	std::string line;
	std::string header;
	std::string faceinfo;

	int poly[4];
	int uv[4];
	int normal[4];

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
			is >> v.x >> v.y >> v.z;
			mesh->verts().push_back(v);
		}
		else if (header == "vn") {
			is >> v.x >> v.y >> v.z;
			mesh->normals().push_back(v);
		}
		else if (header == "f") {
			int i = 0;

			while (is >> faceinfo) {
				auto s1 = faceinfo.find_first_of('/', 0);
				auto s2 = faceinfo.find_first_of('/', s1 + 1);

				poly[i] = std::stoi(faceinfo.substr(0, s1)) - 1;

				if (m_num_uvs > 0) {
					uv[i] = std::stoi(faceinfo.substr(s1 + 1, (s2 - s1) - 1)) - 1;
				}

				if (m_num_normals > 0) {
					normal[i] = std::stoi(faceinfo.substr(s2 + 1)) - 1;
				}

				++i;
			}

			if (i == 3) {
				mesh->tris().emplace_back(glm::ivec3{ poly[0], poly[1], poly[2] });
			}
			else if (i == 4) {
				mesh->quads().emplace_back(glm::ivec4{ poly[0], poly[1], poly[2], poly[3] });
			}
		}
		else if (header == "o") {
			is >> faceinfo;
			mesh->name(faceinfo.c_str());
		}
		else if (header == "g") {
			is >> faceinfo;
			mesh->name(faceinfo.c_str());
		}
	}

	fp_in.close();

	mesh->update();
	mesh->generateGPUData();

	setOutputPrimitive("Mesh", mesh);
}

bool NodeMeshOBJRead::prereadObj(const std::string &filename)
{
	m_num_verts = 0;
	m_num_normals = 0;
	m_num_uvs = 0;
	m_num_polys = 0;

	std::ifstream fp_in;
	fp_in.open(filename.c_str());

	if (!fp_in.is_open()) {
		return false;
	}

	while (fp_in.good()) {
		std::string line;
		std::getline(fp_in, line);

		if (line.empty()) {
			continue;
		}

		const auto &header = line.substr(0, 2);

		if (header.compare("v ") == 0) {
			++m_num_verts;
		}
		else if (header.compare("vt") == 0) {
			++m_num_normals;
		}
		else if (header.compare("vn") == 0) {
//			++m_num_uvs;
		}
		else if (header.compare("f ") == 0) {
			++m_num_polys;
		}
	}

	fp_in.close();
	return true;
}

static Node *new_obj_read_node()
{
	return new NodeMeshOBJRead;
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	factory->registerType("Mesh", NODE_NAME, new_obj_read_node);
}

}
