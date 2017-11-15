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

#include <openvdb/tools/DensityAdvect.h>

template <typename GridType>
void advect(const typename GridType &phiN,
            typename GridType &phiN1,
            openvdb::Vec3SGrid velocity,
            float dt)
{
	using namespace openvdb;

	tools::DensityAdvection advector(velocity);
	advector.setIntegrationOrder(1);

	/* phi^_n+1 = L phi_n */
	typename GridType::Ptr phiHatN1 = advector.advect(phiN, dt);

	/* phi^_n = L^R phi^_n+1 */
	typename GridType::Ptr phiHatN = advector.advect(*phiHatN1, -dt);

	/* phiBar_n = (3 phi_n ­ phi^n ) / 2 */
	typename GridType phiBarN;
	typename GridType::Accessor phiBarN_acc = phiBarN->getAccessor();

	for (typename TreeType::LeafIter lit = phiHatN->tree().beginLeaf(); lit; ++lit) {
		typedef typename TreeType::LeafNodeType LeafT;
		LeafT &leaf = *lit;

		for (typename LeafT::ValueAllIter it = leaf.beginValueAll(); it; ++it) {
			auto coord = it.getCoord();
			auto value = 1.5f * phiN->tree().getValue(coord) - 0.5f * phiHatN->tree().getValue(coord);

			phiBarN_acc.setValue(coord, value);
        }
	}

	/* phi_n+1 = L phiBar_n */
	phiN1 = *advector.advect(*phiBarN, dt);
}
