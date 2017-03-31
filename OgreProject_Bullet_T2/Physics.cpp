#include "Physics.h"

namespace ViRus
{
	Ogre::SceneManager * Hittable::ptr_scn_mgr = nullptr;

	Hittable::~Hittable()
	{
		if (callback)
			(callback)(this);

		delete body;
		delete shape;

		if (scene)
		{
			// Destroy all the attached objects
			Ogre::SceneNode::ObjectIterator itObject = scene->getAttachedObjectIterator();

			while (itObject.hasMoreElements())
			{
				Ogre::MovableObject* pObject = static_cast<Ogre::MovableObject*>(itObject.getNext());
				scene->getCreator()->destroyMovableObject(pObject);
			}

			if (ptr_scn_mgr)
				ptr_scn_mgr->destroySceneNode(scene);
		}
	}

	void HitProjectile::hit(Hittable & h)
	{
		switch (h.type)
		{
			case HittableType::ENEMY:
			{
				HitCharacter *ptr_character = dynamic_cast<HitCharacter *>(&h);
				if (ptr_character)
					ptr_character->takeDamage(dmg);
				isFinished = true;
				break;
			}

			case HittableType::OBSTACLE:
			{
				//isFinished = true;
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

	HitMap::~HitMap()
	{
		clear_all();
	}

	void HitMap::add_hittable(btCollisionObject &c, Hittable &h)
	{
		hittables.insert(std::pair<btCollisionObject *, Hittable *>(&c, &h));
	}
	void HitMap::handle_collision(btCollisionObject * a, btCollisionObject * b)
	{

		//Check that none of the pointers are null
		if (a&&b)
		{
			Hittable *ptr_a = nullptr, *ptr_b = nullptr;

			//Get the hittables
			auto it_a = hittables.find(a);
			if (it_a != hittables.end())
				ptr_a = it_a->second;
			auto it_b = hittables.find(b);
			if (it_b != hittables.end())
				ptr_b = it_b->second;

			//Check that both hittables were found
			if (ptr_a&&ptr_b)
			{
				//Hit eachother
				ptr_a->hit(*ptr_b);
				ptr_b->hit(*ptr_a);

				//Check if either of them needs to be deleted
				if (ptr_a->finished())
				{
					clean_up.insert(a);
				}

				if (ptr_b->finished())
				{
					clean_up.insert(b);
				}
			}
		}
	}

	void HitMap::clean_queued()
	{
		for (btCollisionObject *obj : clean_up)
		{
			delete obj;
			hittables.erase(obj);
		}

		clean_up.clear();
	}

	void HitMap::clear_all()
	{
		for (auto &it : hittables)
		{
			delete it.second;
		}

		hittables.clear();
	}
}