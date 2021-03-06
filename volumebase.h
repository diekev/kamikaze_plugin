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

#pragma once

#include <ego/bufferobject.h>
#include <ego/program.h>
#include <ego/texture.h>
#include <kamikaze/primitive.h>

#include <openvdb/openvdb.h>

class TreeTopology {
	numero7::ego::BufferObject::Ptr m_buffer_data;
	numero7::ego::Program m_program;
	size_t m_elements;

public:
	explicit TreeTopology(openvdb::GridBase::ConstPtr grid);
	~TreeTopology() = default;

	void render(const ViewerContext * const context);
};

class VDBVolume : public Primitive {
	std::unique_ptr<TreeTopology> m_topology;

	RenderBuffer *m_renderbuffer = nullptr;

//	numero7::ego::BufferObject::Ptr m_buffer_data;
//	numero7::ego::Program m_program;
	size_t m_elements;

	openvdb::GridBase::Ptr m_grid;
	openvdb::Mat4R m_volume_matrix;  /* original volume matrix */

	bool m_draw_topology = false;
	bool m_need_draw_update = false;
	int m_storage;

	/* for volume rendering */

//	numero7::ego::Texture3D::Ptr m_volume_texture;
//	numero7::ego::Texture1D::Ptr m_transfer_texture;

	int m_num_slices = 128;

	int m_axis = -1;
	float m_value_scale = 1.0f; // scale of the values contained in the grid (1 / (max - min))
	bool m_use_lut = false;
	char m_num_textures = 0;

public:
	VDBVolume() = default;
	explicit VDBVolume(openvdb::GridBase::Ptr grid);
	~VDBVolume();

	void setGrid(openvdb::GridBase::Ptr grid);

	openvdb::GridBase::Ptr getGridPtr()
	{
		return m_grid;
	}

	openvdb::GridBase &getGrid()
	{
		return *m_grid;
	}

	const openvdb::GridBase &getGrid() const
	{
		return *m_grid;
	}

	int storage() const;

	Primitive *copy() const override;

	void update() override;

	void prepareRenderData() override;
	void render(const ViewerContext &context) override;
	void setCustomUIParams(ParamCallback *cb);
	void computeBBox(glm::vec3 &min, glm::vec3 &max) override;

	static void registerSelf(PrimitiveFactory *factory);

	static size_t id;
	size_t typeID() const override;

private:
	void loadShader();
	void slice(const glm::vec3 &view_dir);
};

inline bool is_level_set(VDBVolume *vol)
{
	return vol->getGridPtr()->getGridClass() == openvdb::GridClass::GRID_LEVEL_SET;
}

bool is_vector_grid(VDBVolume *vol);

void build_vdb_prim(PrimitiveCollection *collection, openvdb::GridBase::Ptr grid);
