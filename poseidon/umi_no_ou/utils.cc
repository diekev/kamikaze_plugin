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

#include <openvdb/openvdb.h>
#include <sys/time.h>

#include "utils.h"

double time_dt()
{
	struct timeval now;
	gettimeofday(&now, nullptr);

	return now.tv_sec + now.tv_usec*1e-6;
}

void write_to_disk(const SimulationFields &fields, const int frame)
{
	using namespace openvdb;

	std::stringstream ss;
	ss << frame;
	std::string num(ss.str());
	num.insert(num.begin(), 1 - (num.size() - 1), '0');

	std::string path("/home/kevin/src/test_files/poseidon/");

	io::File file(path + "test_" + num + ".vdb");
	GridPtrVec grids;

	grids.push_back(fields.density->deepCopy());
	grids.push_back(fields.velocity->deepCopy());
	grids.push_back(fields.temperature->deepCopy());
	grids.push_back(fields.flags->deepCopy());
	grids.push_back(fields.pressure->deepCopy());
	grids.push_back(fields.obstacle->deepCopy());
//	grids.push_back(fields.pindex->deepCopy());

	file.setCompression(io::COMPRESS_BLOSC);

	file.write(grids);
}
