#include "Physics.h"

namespace ViRus
{
	Ogre::SceneManager * Hittable::ptr_scn_mgr = nullptr;


	void HitProjectile::hit(Hittable & h)
	{
		switch (h.type)
		{
			case HittableType::ENEMY:
			{
				HitCharacter *ptr_character = dynamic_cast<HitCharacter *>(&h);
				if (ptr_character)
					ptr_character->takeDamage(dmg);
			}

			case HittableType::OBSTACLE:
			{
				isFinished = true;
				break;
			}

			default:
				break;
		}
	}
	void HitCharacter::hit(Hittable & h)
	{
		switch (h.type)
		{
			case HittableType::PLAYER:
			{
				if (type == HittableType::ENEMY)
				{
					HitCharacter *ptr_character = dynamic_cast<HitCharacter *>(&h);
					if (ptr_character)
						ptr_character->takeDamage(dmg);
				}
			}
		}
	}
}