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
 * The Original Code is Copyright (C) 2017 Kévin Dietrich.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

#include <kamikaze/operateur.h>

static constexpr auto NOM_OPERATEUR = "Poseidon Gaz";
static constexpr auto AIDE_OPERATEUR = "";

class OperateurPoseidonGaz : public Operateur {
	PrimitiveCollection *m_collection_original = nullptr;
	PrimitiveCollection *m_derniere_collection = nullptr;

	int m_image_debut = 0;

public:
	OperateurPoseidonGaz(Noeud *noeud, const Context &contexte)
		: Operateur(noeud, contexte)
	{
		entrees(2);
		sorties(1);
	}

	~OperateurPoseidonGaz()
	{
		delete m_collection_original;
		delete m_derniere_collection;
	}

	const char *nom_entree(size_t index) override
	{
		if (index == 0) {
			return "Sources";
		}

		return "Obstacles";
	}

	const char *nom_sortie(size_t /*index*/) override
	{
		return "Sortie";
	}

	const char *nom() override { return NOM_OPERATEUR; }

	void execute(const Context &contexte, double temps) override
	{
		if (temps == m_image_debut) {
			m_collection->free_all();
			entree(0)->requiers_collection(m_collection, contexte, temps);

			delete m_collection_original;
			m_collection_original = m_collection->copy();
		}
		else {
			m_collection = m_derniere_collection->copy();
		}

		execute_algorithme(contexte, temps);

		/* Sauvegarde la collection */
		delete m_derniere_collection;
		m_derniere_collection = m_collection->copy();
	}

	void execute_algorithme(const Context &contexte, double temps)
	{
	}
};

extern "C" {

void nouvel_operateur_kamikaze(UsineOperateur *usine)
{
	usine->enregistre_type(
				NOM_OPERATEUR,
				cree_description<OperateurPoseidonGaz>(
					NOM_OPERATEUR, AIDE_OPERATEUR, "Physique"));
}

}
