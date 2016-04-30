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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2015 Kévin Dietrich.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "volumebase.h"

#include <ego/utils.h>
#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>

#include <kamikaze/context.h>
#include <kamikaze/paramfactory.h>

#include <openvdb/tools/GridTransformer.h>

#include "util_openvdb.h"
#include "util_openvdb_process.h"

const int MAX_SLICES = 512;

struct TreeTopologyOp {
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> colors;
	std::vector<unsigned int> indices;

	template <typename GridType>
	void operator()(typename GridType::ConstPtr grid)
	{
		using openvdb::Index64;
		using openvdb::math::Vec3d;
		using openvdb::math::CoordBBox;

		typedef typename GridType::TreeType TreeType;

		Index64 nodeCount = grid->tree().leafCount() + grid->tree().nonLeafCount();
	    const Index64 N = nodeCount * 8;

		vertices.resize(N);
		colors.resize(N);
		indices.resize(N * 3);

		CoordBBox bbox;
		int idx = 0, count = 0, col_offset = 0, idx_offset = 0;

		glm::vec3 node_colors[] = {
		    glm::vec3(0.045f, 0.045f, 0.045f),         // root
		    glm::vec3(0.0432f, 0.33f, 0.0411023f),     // first internal node level
		    glm::vec3(0.871f, 0.394f, 0.01916f),       // intermediate internal node levels
		    glm::vec3(0.00608299f, 0.279541f, 0.625f)  // leaf nodes
		};

		for (typename TreeType::NodeCIter iter = grid->tree().cbeginNode(); iter; ++iter) {
	        iter.getBoundingBox(bbox);

	        /* Nodes are rendered as cell-centered */
	        Vec3d min(bbox.min().asVec3d() - Vec3d(0.5));
	        Vec3d max(bbox.max().asVec3d() + Vec3d(0.5));
			min = grid->indexToWorld(min);
			max = grid->indexToWorld(max);

			const glm::vec3 corners[8] = {
			    glm::vec3(min.x(), min.y(), min.z()),
			    glm::vec3(min.x(), min.y(), max.z()),
			    glm::vec3(max.x(), min.y(), max.z()),
			    glm::vec3(max.x(), min.y(), min.z()),
			    glm::vec3(min.x(), max.y(), min.z()),
			    glm::vec3(min.x(), max.y(), max.z()),
			    glm::vec3(max.x(), max.y(), max.z()),
			    glm::vec3(max.x(), max.y(), min.z()),
			};

	        vertices[count++] = corners[0];
			vertices[count++] = corners[1];
			vertices[count++] = corners[2];
			vertices[count++] = corners[3];
			vertices[count++] = corners[4];
			vertices[count++] = corners[5];
			vertices[count++] = corners[6];
			vertices[count++] = corners[7];

	        // edge 1
	        indices[idx_offset++] = GLuint(idx);
	        indices[idx_offset++] = GLuint(idx + 1);
	        // edge 2
	        indices[idx_offset++] = GLuint(idx + 1);
	        indices[idx_offset++] = GLuint(idx + 2);
	        // edge 3
	        indices[idx_offset++] = GLuint(idx + 2);
	        indices[idx_offset++] = GLuint(idx + 3);
	        // edge 4
	        indices[idx_offset++] = GLuint(idx + 3);
	        indices[idx_offset++] = GLuint(idx);
	        // edge 5
	        indices[idx_offset++] = GLuint(idx + 4);
	        indices[idx_offset++] = GLuint(idx + 5);
	        // edge 6
	        indices[idx_offset++] = GLuint(idx + 5);
	        indices[idx_offset++] = GLuint(idx + 6);
	        // edge 7
	        indices[idx_offset++] = GLuint(idx + 6);
	        indices[idx_offset++] = GLuint(idx + 7);
	        // edge 8
	        indices[idx_offset++] = GLuint(idx + 7);
	        indices[idx_offset++] = GLuint(idx + 4);
	        // edge 9
	        indices[idx_offset++] = GLuint(idx);
	        indices[idx_offset++] = GLuint(idx + 4);
	        // edge 10
	        indices[idx_offset++] = GLuint(idx + 1);
	        indices[idx_offset++] = GLuint(idx + 5);
	        // edge 11
	        indices[idx_offset++] = GLuint(idx + 2);
	        indices[idx_offset++] = GLuint(idx + 6);
	        // edge 12
	        indices[idx_offset++] = GLuint(idx + 3);
	        indices[idx_offset++] = GLuint(idx + 7);

	        idx += 8;

			const int level = iter.getLevel();
	        glm::vec3 color = node_colors[(level == 0) ? 3 : (level == 1) ? 2 : 1];
			for (int i(0); i < 8; ++i) {
				colors[col_offset++] = color;
			}
	    }
	}
};

TreeTopology::TreeTopology(openvdb::GridBase::ConstPtr grid)
    : m_buffer_data(ego::BufferObject::create())
{
	m_program.load(ego::VERTEX_SHADER, ego::util::str_from_file("shaders/tree_topology.vert"));
	m_program.load(ego::FRAGMENT_SHADER, ego::util::str_from_file("shaders/tree_topology.frag"));

	m_program.createAndLinkProgram();

	m_program.enable();
	{
		m_program.addAttribute("vertex");
		m_program.addAttribute("color");
		m_program.addUniform("MVP");
	}
	m_program.disable();

	TreeTopologyOp op;
	process_typed_grid(grid, get_grid_storage(*grid), op);

	m_elements = op.indices.size();

	m_buffer_data->bind();
	m_buffer_data->generateVertexBuffer(&op.vertices[0][0], sizeof(glm::vec3) * op.vertices.size());
	m_buffer_data->generateIndexBuffer(&op.indices[0], sizeof(GLuint) * m_elements);
	m_buffer_data->attribPointer(m_program["vertex"], 3);
	m_buffer_data->generateNormalBuffer(&op.colors[0][0], sizeof(glm::vec3) * op.colors.size());
	m_buffer_data->attribPointer(m_program["color"], 3);
	m_buffer_data->unbind();
}

void TreeTopology::render(ViewerContext *context)
{
	if (m_program.isValid()) {
		m_program.enable();
		m_buffer_data->bind();

		glUniformMatrix4fv(m_program("MVP"), 1, GL_FALSE, glm::value_ptr(context->MVP()));
		glDrawElements(GL_LINES, m_elements, GL_UNSIGNED_INT, nullptr);

		m_buffer_data->unbind();
		m_program.disable();
	}
}

VDBVolume::VDBVolume(openvdb::GridBase::Ptr grid)
{
	setGrid(grid);
}

void VDBVolume::setGrid(openvdb::GridBase::Ptr grid)
{
	m_grid = grid;
	m_volume_matrix = m_grid->transform().baseMap()->getAffineMap()->getMat4();
	m_storage = get_grid_storage(*m_grid);

	const auto &bbox = m_grid->evalActiveVoxelBoundingBox();

	const auto &ws_bbox = m_grid->transform().indexToWorld(bbox);
	const auto &min = ws_bbox.min();
	const auto &max = ws_bbox.max();

	m_min = convertOpenVDBVec(min);
	m_max = convertOpenVDBVec(max);
	m_dimensions = (m_max - m_min);

	updateMatrix();

	m_bbox = std::unique_ptr<Cube>(new Cube(m_min, m_max));
	m_buffer_data = ego::BufferObject::create();
	m_topology = std::unique_ptr<TreeTopology>(new TreeTopology(grid));

	m_need_draw_update = true;
}

void VDBVolume::prepareRenderData()
{
	if (!m_grid.get() || m_grid->empty()) {
		return;
	}

	if (!m_need_data_update) {
		return;
	}

	if (m_grid->getGridClass() == openvdb::GridClass::GRID_LEVEL_SET) {
		loadShader();

		VolumeMesherOp op;
		op.inv_mat = m_inv_matrix;

		process_grid_real(m_grid, m_storage, op);

		m_elements = op.indices.size();

		m_buffer_data.reset(new ego::BufferObject());
		m_buffer_data->bind();
		m_buffer_data->generateVertexBuffer(&op.vertices[0][0], op.vertices.size() * sizeof(glm::vec3));
		m_buffer_data->generateIndexBuffer(&op.indices[0], m_elements * sizeof(GLuint));
		m_buffer_data->attribPointer(m_program["vertex"], 3);
		m_buffer_data->generateNormalBuffer(&op.normals[0], op.normals.size() * sizeof(GLfloat));
		m_buffer_data->attribPointer(m_program["normal"], 3);
		m_buffer_data->unbind();

		ego::util::GPU_check_errors("Unable to create level set buffer");
	}
	else {
		m_elements = m_num_slices * 6;

		/* Get resolution & copy data */
		openvdb::math::CoordBBox bbox = m_grid->evalActiveVoxelBoundingBox();

		SparseToDenseOp op;
		op.bbox = bbox;
		op.data = new GLfloat[bbox.volume()];

		process_grid_real(m_grid, m_storage, op);

		m_volume_texture = ego::Texture3D::create(0);
		m_volume_texture->bind();
		m_volume_texture->setType(GL_FLOAT, GL_RED, GL_RED);
		m_volume_texture->setMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
		m_volume_texture->setWrapping(GL_CLAMP_TO_BORDER);
		m_volume_texture->fill(op.data, bbox.dim().asPointer());
		m_volume_texture->generateMipMap(0, 4);
		m_volume_texture->unbind();

		ego::util::GPU_check_errors("Unable to create 3D texture");

		loadShader();
	}

	m_need_data_update = false;
}

int VDBVolume::storage() const
{
	return m_storage;
}

void VDBVolume::update()
{
	if (m_grid.get() && m_need_update) {
		updateMatrix();
		updateGridTransform();
		m_bbox.reset(new Cube(m_min, m_max));
		m_need_update = false;
	}
}

void VDBVolume::updateGridTransform()
{
	typedef openvdb::math::AffineMap AffineMap;
	typedef openvdb::math::Transform Transform;

	const openvdb::Vec3R pos = convertGLMVec(m_pos);
	const openvdb::Vec3R scale = convertGLMVec(m_scale);

	openvdb::Mat4R mat(openvdb::Mat4R::identity());
    mat.preTranslate(pos);
    mat.preRotate(openvdb::math::X_AXIS, glm::radians(m_rotation[0]));
    mat.preRotate(openvdb::math::Y_AXIS, glm::radians(m_rotation[1]));
    mat.preRotate(openvdb::math::Z_AXIS, glm::radians(m_rotation[2]));
    mat.preScale(scale);
    mat.preTranslate(-pos);
    mat.preTranslate(pos);

	openvdb::math::AffineMap map(mat), original_map(m_volume_matrix);
	AffineMap::Ptr compound(new AffineMap(original_map, map));

	m_grid->setTransform(Transform::Ptr(new Transform(openvdb::math::simplify(compound))));

	// TODO: topology is only updated if drawn
	if (m_draw_topology) {
		m_topology.reset(new TreeTopology(m_grid));
	}
}

Primitive *VDBVolume::copy() const
{
	return new VDBVolume(m_grid->deepCopyGrid());
}

void VDBVolume::loadShader()
{
	if (m_grid->getGridClass() == openvdb::GridClass::GRID_LEVEL_SET) {
		m_program.load(ego::VERTEX_SHADER, ego::util::str_from_file("shaders/object.vert"));
		m_program.load(ego::FRAGMENT_SHADER, ego::util::str_from_file("shaders/object.frag"));
		m_program.createAndLinkProgram();

		m_program.enable();
		{
			m_program.addAttribute("vertex");
			m_program.addAttribute("normal");
			m_program.addUniform("matrix");
			m_program.addUniform("MVP");
			m_program.addUniform("N");
			m_program.addUniform("for_outline");
		}
		m_program.disable();
	}
	else {
		m_program.load(ego::VERTEX_SHADER, ego::util::str_from_file("shaders/volume.vert"));
		m_program.load(ego::FRAGMENT_SHADER, ego::util::str_from_file("shaders/volume.frag"));

		m_program.createAndLinkProgram();

		m_program.enable();
		{
			m_program.addAttribute("vertex");
			m_program.addUniform("MVP");
			m_program.addUniform("offset");
			m_program.addUniform("volume");
			m_program.addUniform("lut");
			m_program.addUniform("use_lut");
			m_program.addUniform("scale");
			m_program.addUniform("matrix");

			glUniform1i(m_program("volume"), m_volume_texture->number());
//			glUniform1i(m_program("lut"), m_transfer_texture->number());
			glUniform1f(m_program("scale"), m_value_scale);
		}
		m_program.disable();

		const auto &vsize = MAX_SLICES * 4 * sizeof(glm::vec3);
		const auto &isize = MAX_SLICES * 6 * sizeof(GLuint);

		m_buffer_data->bind();
		m_buffer_data->generateVertexBuffer(nullptr, vsize);
		m_buffer_data->generateIndexBuffer(nullptr, isize);
		m_buffer_data->attribPointer(m_program["vertex"], 3);
		m_buffer_data->unbind();
	}
}

void VDBVolume::slice(const glm::vec3 &view_dir)
{
	auto axis = axis_dominant_v3_single(glm::value_ptr(view_dir));

	if (m_axis == axis) {
		return;
	}

	m_axis = axis;
	auto depth = m_min[m_axis];
	auto slice_size = m_dimensions[m_axis] / m_num_slices;

	/* always process slices in back to front order! */
	if (view_dir[m_axis] > 0.0f) {
		depth = m_max[m_axis];
		slice_size = -slice_size;
	}

	const glm::vec3 vertices[3][4] = {
	    {
	        glm::vec3(0.0f, m_min[1], m_min[2]),
	        glm::vec3(0.0f, m_max[1], m_min[2]),
	        glm::vec3(0.0f, m_max[1], m_max[2]),
	        glm::vec3(0.0f, m_min[1], m_max[2])
	    },
	    {
	        glm::vec3(m_min[0], 0.0f, m_min[2]),
	        glm::vec3(m_min[0], 0.0f, m_max[2]),
	        glm::vec3(m_max[0], 0.0f, m_max[2]),
	        glm::vec3(m_max[0], 0.0f, m_min[2])
	    },
	    {
	        glm::vec3(m_min[0], m_min[1], 0.0f),
	        glm::vec3(m_min[0], m_max[1], 0.0f),
	        glm::vec3(m_max[0], m_max[1], 0.0f),
	        glm::vec3(m_max[0], m_min[1], 0.0f)
	    }
	};

	GLuint *indices = new GLuint[m_elements];
	int idx = 0, idx_count = 0;

	std::vector<glm::vec3> points;
	points.reserve(m_num_slices * 4);

	for (auto slice(0); slice < m_num_slices; slice++) {
		glm::vec3 v0 = vertices[m_axis][0];
		glm::vec3 v1 = vertices[m_axis][1];
		glm::vec3 v2 = vertices[m_axis][2];
		glm::vec3 v3 = vertices[m_axis][3];

		v0[m_axis] = depth;
		v1[m_axis] = depth;
		v2[m_axis] = depth;
		v3[m_axis] = depth;

		points.push_back(v0 * glm::mat3(m_inv_matrix));
		points.push_back(v1 * glm::mat3(m_inv_matrix));
		points.push_back(v2 * glm::mat3(m_inv_matrix));
		points.push_back(v3 * glm::mat3(m_inv_matrix));

		indices[idx_count++] = idx + 0;
		indices[idx_count++] = idx + 1;
		indices[idx_count++] = idx + 2;
		indices[idx_count++] = idx + 0;
		indices[idx_count++] = idx + 2;
		indices[idx_count++] = idx + 3;

		depth += slice_size;
		idx += 4;
	}

	m_buffer_data->updateVertexBuffer(&vertices[0][0], points.size() * sizeof(glm::vec3));
	m_buffer_data->updateIndexBuffer(indices, idx_count * sizeof(GLuint));

	delete [] indices;
}

void VDBVolume::render(ViewerContext *context, const bool for_outline)
{
	if (!m_grid.get() || m_grid->empty()) {
		return;
	}

	if (m_draw_topology) {
		m_topology->render(context);
	}

	if (m_grid->getGridClass() == openvdb::GridClass::GRID_LEVEL_SET) {
		if (m_program.isValid()) {
			m_program.enable();
			m_buffer_data->bind();

			glUniformMatrix4fv(m_program("matrix"), 1, GL_FALSE, glm::value_ptr(m_matrix));
			glUniformMatrix4fv(m_program("MVP"), 1, GL_FALSE, glm::value_ptr(context->MVP()));
			glUniformMatrix3fv(m_program("N"), 1, GL_FALSE, glm::value_ptr(context->normal()));
			glUniform1i(m_program("for_outline"), for_outline);
			glDrawElements(m_draw_type, m_elements, GL_UNSIGNED_INT, nullptr);

			m_buffer_data->unbind();
			m_program.disable();
		}
	}
	else {
		slice(context->view());

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (m_program.isValid()) {
			m_program.enable();
			m_buffer_data->bind();
			m_volume_texture->bind();
//			m_transfer_texture->bind();

			auto min = m_min * glm::mat3(m_inv_matrix);
			glUniform3fv(m_program("offset"), 1, &min[0]);
			glUniformMatrix4fv(m_program("MVP"), 1, GL_FALSE, glm::value_ptr(context->MVP()));
			glUniformMatrix4fv(m_program("matrix"), 1, GL_FALSE, glm::value_ptr(m_matrix));
//			glUniform1i(m_program("use_lut"), m_use_lut);
			glDrawElements(m_draw_type, m_elements, GL_UNSIGNED_INT, nullptr);

//			m_transfer_texture->unbind();
			m_volume_texture->unbind();
			m_buffer_data->unbind();
			m_program.disable();
		}

		glDisable(GL_BLEND);
	}
}

void VDBVolume::setCustomUIParams(ParamCallback *cb)
{
	if (!m_grid.get() || m_grid->empty()) {
		return;
	}

	bool_param(cb, "Draw Topology", &m_draw_topology, m_draw_topology);

	if (m_grid->getGridClass() != openvdb::GridClass::GRID_LEVEL_SET) {
		int_param(cb, "Slices", &m_num_slices, 1, 512, m_num_slices);
		bool_param(cb, "Use LUT", &m_use_lut, m_use_lut);
	}
}

void VDBVolume::computeBBox(glm::vec3 &min, glm::vec3 &max)
{
	if (!m_grid.get() || m_grid->empty()) {
		min = glm::vec3(0.0f);
		max = glm::vec3(0.0f);
		return;
	}

	auto bbox = m_grid->evalActiveVoxelBoundingBox();
	auto wbbox = m_grid->transform().indexToWorld(bbox);

	min = glm::vec3(wbbox.min()[0], wbbox.min()[1], wbbox.min()[2]);
	max = glm::vec3(wbbox.max()[0], wbbox.max()[1], wbbox.max()[2]);
}

static Primitive *create_vdb_volume()
{
	return new VDBVolume();
}

extern "C" {

void new_kamikaze_prims(PrimitiveFactory *factory)
{
	/* Avoid double registration. */
	if (factory->registered("OpenVDB Volume")) {
		return;
	}

	factory->registerType("OpenVDB Volume", create_vdb_volume);
}

}

#if 0
void Volume::loadTransferFunction()
{
	/* transfer function (lookup table) color values */
	const glm::vec3 jet_values[12] = {
	    glm::vec3(1.0f, 0.0f, 0.0f),
		glm::vec3(1.0f, 0.0f, 0.5f),
		glm::vec3(1.0f, 0.0f, 1.0f),

		glm::vec3(0.5f, 0.0f, 1.0f),
		glm::vec3(0.0f, 0.5f, 1.0f),
		glm::vec3(0.0f, 1.0f, 1.0f),

		glm::vec3(0.0f, 1.0f, 0.5f),
		glm::vec3(0.0f, 1.0f, 0.0f),
		glm::vec3(0.5f, 1.0f, 0.0f),

		glm::vec3(1.0f, 1.0f, 0.0f),
		glm::vec3(1.0f, 0.5f, 0.0f),
		glm::vec3(1.0f, 0.0f, 0.0f),
	};

	int size = 256;
	float data[size][3];
	int indices[12];

	for (int i = 0; i < 12; ++i) {
		indices[i] = i * 21;
	}

	/* for each adjacent pair of colors, find the difference in the RGBA values
	 * and then interpolate */
	for (int j = 0; j < 12 - 1; ++j) {
		auto color_diff = jet_values[j + 1] - jet_values[j];
		auto index = indices[j + 1] - indices[j];
		auto inc = color_diff / static_cast<float>(index);

		for (int i = indices[j] + 1; i < indices[j + 1]; ++i) {
			data[i][0] = jet_values[j].r + i * inc.r;
			data[i][1] = jet_values[j].g + i * inc.g;
			data[i][2] = jet_values[j].b + i * inc.b;
		}
	}

	m_transfer_texture = ego::Texture1D::create(m_num_textures++);
	m_transfer_texture->bind();
	m_transfer_texture->setType(GL_FLOAT, GL_RGB, GL_RGB);
	m_transfer_texture->setMinMagFilter(GL_LINEAR, GL_LINEAR);
	m_transfer_texture->setWrapping(GL_REPEAT);
	m_transfer_texture->fill(&data[0][0], &size);
	m_transfer_texture->unbind();
}

bool LevelSet::intersectLS(const Ray &/*ray*/, Brush */*brush*/)
{
	using namespace openvdb;

	openvdb::math::Vec3d P(ray.pos.x, ray.pos.y, ray.pos.z);
	openvdb::math::Vec3d D(ray.dir.x, ray.dir.y, ray.dir.z);
	D.normalize();

	ray_t vray(P, D, 1e-5, std::numeric_limits<double>::max());

	openvdb::math::Vec3d position;

	m_isector.reset(new isector_t(*m_level_set));
	if (m_isector->intersectsWS(vray, position)) {
		math::Coord ijk = m_level_set->transform().worldToIndexNodeCentered(position);

		if (brush->tool() == BRUSH_TOOL_DRAW) {
			do_sculpt_draw(*m_level_set, brush, ijk, m_voxel_size);
		}
		else {
			do_sculpt_smooth(*m_level_set, brush, ijk, m_voxel_size);
		}

		m_topology_changed = true;

		if (m_draw_topology) {
			m_topology.reset(new TreeTopology(m_level_set));
		}

		generateMesh(true);
	}

	return false;
}
#endif

bool is_vector_grid(VDBVolume *vol)
{
	return is_elem(vol->storage(), GRID_STORAGE_VEC3D, GRID_STORAGE_VEC3S, GRID_STORAGE_VEC3I);
}
