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

#include <openvdb/tools/GridTransformer.h>

#include "util_openvdb.h"
#include "util_openvdb_process.h"

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

VolumeBase::VolumeBase()
    : m_buffer_data(nullptr)
    , m_elements(0)
    , m_topology(nullptr)
    , m_draw_topology(false)
{}

void VolumeBase::setupData(openvdb::GridBase::Ptr grid)
{
	using namespace openvdb;
	using namespace openvdb::math;

	m_grid = grid;
	m_volume_matrix = m_grid->transform().baseMap()->getAffineMap()->getMat4();
	m_voxel_size = m_grid->transform().voxelSize()[0];
	m_topology_changed = false;

	CoordBBox bbox = m_grid->evalActiveVoxelBoundingBox();

	BBoxd ws_bbox = m_grid->transform().indexToWorld(bbox);
	Vec3f min = ws_bbox.min();
	Vec3f max = ws_bbox.max();

	m_min = convertOpenVDBVec(min);
	m_max = convertOpenVDBVec(max);
	m_dimensions = (m_max - m_min);
	updateMatrix();

	m_bbox = std::unique_ptr<Cube>(new Cube(m_min, m_max));
	m_buffer_data = ego::BufferObject::create();
	m_topology = std::unique_ptr<TreeTopology>(new TreeTopology(grid));
}

VolumeBase::VolumeBase(openvdb::GridBase::Ptr grid)
    : VolumeBase()
{
	using namespace openvdb;
	using namespace openvdb::math;

	setupData(grid);
}

void VolumeBase::update()
{
	if (m_need_update) {
		updateMatrix();
		updateGridTransform();
		m_bbox.reset(new Cube(m_min, m_max));
		m_need_update = false;
	}
}

float VolumeBase::voxelSize() const
{
	return m_voxel_size;
}

void VolumeBase::setVoxelSize(const float voxel_size)
{
	if (voxel_size == 0.0f) {
		return;
	}

	if (m_voxel_size != voxel_size) {
		m_voxel_size = voxel_size;
		resampleGridVoxel();
		m_topology_changed = true;
	}
}

void VolumeBase::updateGridTransform()
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

struct ResampleGridOp {
	const float voxel_size;

	explicit ResampleGridOp(const float vsize)
	    : voxel_size(vsize)
	{}

	template <typename GridType>
	void operator()(typename GridType::Ptr grid)
	{
		using namespace openvdb;

		typedef typename GridType::ValueType ValueType;
		typedef math::Transform Transform;

		Transform::Ptr xform = Transform::createLinearTransform(voxel_size);
		typename GridType::Ptr output;

		if (grid->getGridClass() == GRID_LEVEL_SET) {
			const float halfwidth = grid->background() * (1.0f / grid->transform().voxelSize()[0]);
			util::NullInterrupter interrupt;

			output = tools::doLevelSetRebuild(*grid, zeroVal<ValueType>(),
			                                  halfwidth, halfwidth,
			                                  xform.get(), &interrupt);
		}
		else {
			output = GridType::create(grid->background());
			output->setTransform(xform);
			output->setName(grid->getName());
			output->setGridClass(grid->getGridClass());

			tools::resampleToMatch<openvdb::tools::PointSampler>(*grid, *output);
		}

		grid.swap(output);
	}
};

void VolumeBase::resampleGridVoxel()
{
	ResampleGridOp op(m_voxel_size);

	process_grid_real(m_grid, get_grid_storage(*m_grid), op);

	if (m_draw_topology) {
		m_topology.reset(new TreeTopology(m_grid));
	}
}
