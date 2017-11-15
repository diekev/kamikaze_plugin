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

#include "utils.h"

#include <sstream>
#include <sys/time.h>

double time_dt()
{
	struct timeval now;
	gettimeofday(&now, nullptr);

	return now.tv_sec + now.tv_usec*1e-6;
}

std::string get_name_for_frame(const std::string &name, const int frame)
{
	std::stringstream ss;
	ss << frame;

	std::string num(ss.str());
	num.insert(num.begin(), 1 - (num.size() - 1), '0');

	return name + "." + num;
}

#if 0
void print_vel(Particle *p, const std::string &message)
{
	printf("%s vel: %f, %f, %f\n", message.c_str(), p->vel.x(), p->vel.y(), p->vel.z());
	printf("%s pic: %f, %f, %f\n", message.c_str(), p->vel_pic.x(), p->vel_pic.y(), p->vel_pic.z());
	printf("%s flip: %f, %f, %f\n", message.c_str(), p->vel_flip.x(), p->vel_flip.y(), p->vel_flip.z());
}
#endif
