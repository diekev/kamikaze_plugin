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

#include "util_openvdb_process.h"
#include "util_string.h"
#include "volumebase.h"

static auto begin(const openvdb::io::File &file)
{
	return file.beginName();
}

static auto end(const openvdb::io::File &file)
{
	return file.endName();
}

static constexpr auto NODE_NAME = "OpenVDB Read";

class NodeOpenVDBRead : public VDBNode {

public:
	NodeOpenVDBRead();

	void process() override;
	bool update_properties() override;
};

NodeOpenVDBRead::NodeOpenVDBRead()
    : VDBNode(NODE_NAME)
{
	addOutput("output");

	openvdb::initialize();

	add_prop("metadata_only", "Read Metadata Only", property_type::prop_bool);
	set_prop_tooltip("If enabled, output empty grids populated with\n"
	                 "their metadata and transforms only.\n");

	add_prop("filepath", "File Path", property_type::prop_input_file);
	set_prop_tooltip("Select a VDB file.");

	add_prop("grids", "Grid(s)", property_type::prop_list);
	set_prop_default_value_string("*");
	set_prop_tooltip("Names of the grid(s) to read.");

	/* Delayed loading. */
	add_prop("delayload", "Delay Loading", property_type::prop_bool);
	set_prop_default_value_bool(true);
	set_prop_tooltip("Don't allocate memory for or read voxel values until the values\n"
	                 "are actually accessed.\n\n"
		             "Delayed loading can significantly lower memory usage, but\n"
		             "note that viewport visualization of a volume usually requires\n"
		             "the entire volume to be loaded into memory.");

    /* Localization file size slider. */
	add_prop("copylimit", "Copy if smaller than", property_type::prop_float);
	set_prop_default_value_float(0.5f);
	set_prop_min_max(0.0f, 10.0f);
	set_prop_tooltip("When delayed loading is enabled, a file must not be modified on disk before\n"
	                 "it has been fully read.  For safety, files smaller than the given size (in GB)\n"
		             "will be copied to a private, temporary location (either $OPENVDB_TEMP_DIR,\n"
		             "$TMPDIR or a system default temp directory).");
}

bool NodeOpenVDBRead::update_properties()
{
	const auto delayload = eval_bool("delayload");
    set_prop_visible("copylimit", delayload);

	const auto filename = eval_string("filepath");

	if (filename.empty()) {
		return true;
	}

	EnumProperty items;
	items.insert("*", 0);

	try {
		openvdb::io::File file(filename);
        file.open();

        // Loop over the names of all of the grids in the file.
        for (auto iter = file.beginName(); iter != file.endName(); ++iter) {
            // Add the grid's name to the list.
			items.insert(iter.gridName(), 0);
        }

        file.close();
    } catch (...) {}

	for (Property &prop : this->props()) {
		if (prop.name == "grids") {
			prop.enum_items = items;
		}
	}

	return true;
}

void NodeOpenVDBRead::process()
{
	const auto readMetadataOnly = eval_bool("metadata_only");
	const auto filename = eval_string("filepath");
	const auto grids = eval_string("grids");
	const auto delayedLoad = eval_bool("delayload");
    const auto copyMaxBytes = static_cast<openvdb::Index64>(1.0e9 * eval_float("copylimit"));

	openvdb::io::File file(filename);
	openvdb::MetaMap::Ptr fileMetadata;

	try {
		/* Open the VDB file, but don't read any grids yet. */
		file.setCopyMaxBytes(copyMaxBytes);
        file.open(delayedLoad);

		/* Read the file-level metadata. */
        fileMetadata = file.getMetadata();

		if (!fileMetadata) {
			fileMetadata.reset(new openvdb::MetaMap);
		}
	}
	catch (const std::exception &e) {
		this->add_warning(e.what());
		return;
	}

	for (auto iter = file.beginName(); iter != file.endName(); ++iter) {
		/* Skip grids whose names don't match the user-supplied mask. */
		const auto &gridName = iter.gridName();

		if (!find_match(grids, gridName)) {
			continue;
		}

		openvdb::GridBase::Ptr grid;

		if (readMetadataOnly) {
			grid = file.readGridMetadata(gridName);
		}
		else {
			grid = file.readGrid(gridName);
		}

		if (!grid) {
			continue;
		}

		/* Copy file-level metadata into the grid, then create (if necessary)
		 * and set a primitive attribute for each metadata item. */
		for (auto fileMetaIt = fileMetadata->beginMeta(), end = fileMetadata->endMeta();
		     fileMetaIt != end; ++fileMetaIt)
		{
			/* Resolve file- and grid-level metadata name conflictsin favor of
			 * the grid-level metadata. */
			if (openvdb::Metadata::Ptr meta = fileMetaIt->second) {
				const auto name = fileMetaIt->first;

				if (!(*grid)[name]) {
					grid->insertMeta(name, *meta);
				}
			}
		}

		build_vdb_prim(m_collection, grid);
	}

    file.close();
}

extern "C" {

void new_kamikaze_node(NodeFactory *factory)
{
	REGISTER_NODE("VDB", NODE_NAME, NodeOpenVDBRead);
}

}
