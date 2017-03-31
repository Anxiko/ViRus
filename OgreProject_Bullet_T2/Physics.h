#ifndef _VIRUS_PHYSICS_
#define _VIRUS_PHYSICS_


#include "OgreBulletDynamicsRigidBody.h" // for OgreBullet
#include "Shapes/OgreBulletCollisionsStaticPlaneShape.h" // for static planes
#include "Shapes/OgreBulletCollisionsBoxShape.h" // for boxes
#include "BaseApplication.h"
#include <iostream>
#include <fstream>

#include <map>
#include <set>

namespace ViRus
{

	//Class declarations
	class Hittable;//Object that can be hit
	class HitProjectile;//Projectile that can hit objects
	class HitCharacter;//Character that can be hit
	class HitMap;//Map that goes from the physics object to the hittable object

	//Types

	enum class HittableType
	{
		START = 0,
		ENEMY,//Enemy to kill
		PLAYER,//Player controlled by a person
		BULLET,//Bullet fired from a gun
		OBSTACLE,//Inanimated object
		NONE,//Not defined
		END
	};

	//Class definitions

	//Object that can be hit
	class Hittable
	{
		public:

			static Ogre::SceneManager *ptr_scn_mgr;

		public:
			HittableType type;

	protected:
			OgreBulletDynamics::RigidBody *body;
			OgreBulletCollisions::CollisionShape *shape;
			Ogre::SceneNode *scene;
			void(*callback) (Hittable *) = nullptr;


		public:
			Hittable(HittableType itype, OgreBulletDynamics::RigidBody *ibody, OgreBulletCollisions::CollisionShape *ishape, Ogre::SceneNode *iscene)
				:type(itype),body(ibody),shape(ishape),scene(iscene)
			{}

			virtual ~Hittable();

		public:

			//Hit another hittable
			virtual void hit(Hittable &h)
			{}

			//Return true if this hittable is done and can be removed from the map
			virtual bool finished()
			{
				return false;
			}

			//Set the callback to be called before deletion
			void set_callback(void(*icallback) (Hittable *))
			{
				callback = icallback;
			}
	};

	//Projectile
	class HitProjectile : public Hittable
	{

		private:
			bool isFinished = false;//Bullet has hit something, and despawns
			int dmg;//Damage that this bullet deals upon hitting a character

		public:

			HitProjectile(OgreBulletDynamics::RigidBody *ibody, OgreBulletCollisions::CollisionShape *ishape, Ogre::SceneNode *iscene, int idmg=0)
			:Hittable(HittableType::BULLET,ibody,ishape,iscene), dmg(idmg)
			{}

		public:

			//Hit another hittable
			virtual void hit(Hittable &h);

			//Return true if this hittable is done and can be removed from the map
			virtual bool finished()
			{
				return isFinished;
			}
	};

	//Character that can be hit
	class HitCharacter : public Hittable
	{
		private:
			int health;//Health that his character has (below 0, character is dead)
			int dmg;//Damage that this charcter deals (0 for no attacks)

		public:

			HitCharacter(HittableType itype, OgreBulletDynamics::RigidBody *ibody, OgreBulletCollisions::CollisionShape *ishape, Ogre::SceneNode *iscene, int ihealth, int idmg = 0)
			:Hittable(itype, ibody,ishape,iscene), health(ihealth), dmg(idmg)
			{}

		public:

			//Hit another hittable
			virtual void hit(Hittable &h);

			//Return true if this hittable is done and can be removed from the map
			virtual bool finished()
			{
				return health <= 0;
			}

			//Take damage
			void takeDamage(int idmg)
			{
				health -= idmg;
			}
	};


	//Map that goes from the physics object to the hittable object
	class HitMap
	{
		private:

			std::map<btCollisionObject*, Hittable *> hittables;
			std::set<btCollisionObject *> clean_up;

		public:

			~HitMap();

		public:

			//Add a hittable
			void add_hittable(btCollisionObject &c, Hittable &h);

			//Handle collision
			void handle_collision(btCollisionObject *a, btCollisionObject *b);

			//Delete pointers queued for destroctions
			void clean_queued();

		public:

			//Delete all hittables
			void clear_all();
	};

}

#endif // !_VIRUS_PHYSICS_
