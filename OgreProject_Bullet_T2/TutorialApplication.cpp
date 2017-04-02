/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "TutorialApplication.h"

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
	constexpr float PENGUIN_SCALING = 0.1;

	ViRus::Hittable::ptr_scn_mgr = mSceneMgr;

	mNumEntitiesInstanced = 0; // how many shapes are created

							   // Start Bullet
	mWorld = new OgreBulletDynamics::DynamicsWorld(mSceneMgr,
		Ogre::AxisAlignedBox(Ogre::Vector3(-10000, -10000, -10000), Ogre::Vector3(10000, 10000, 10000)), //aligned box for Bullet
		Ogre::Vector3(0, -9.81, 0)); // gravity vector for Bullet

									 // add Debug info display tool
	debugDrawer = new OgreBulletCollisions::DebugDrawer();
	debugDrawer->setDrawWireframe(true); // we want to see the Bullet shapes

	mWorld->setDebugDrawer(debugDrawer);
	mWorld->setShowDebugShapes(true); // enable it if you want to see the Bullet shapes
	Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode("debugDrawer", Ogre::Vector3::ZERO);
	node->attachObject(static_cast <Ogre::SimpleRenderable *> (debugDrawer));

	// Set ambient light
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

	// Create a light
	Ogre::Light* l = mSceneMgr->createLight("MainLight");
	l->setPosition(20, 80, 50);

	// Set camera position
	mCamera->setPosition(Ogre::Vector3(0, 15, 50));

	// Define a floor plane mesh
	Ogre::Plane floorPlane;
	floorPlane.normal = Ogre::Vector3::UNIT_Y;
	floorPlane.d = 0.0;
	Ogre::MeshManager::getSingleton().createPlane("FloorPlane",
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		floorPlane, 1000, 1000, 20, 20, true, 1, 50, 50, Ogre::Vector3::UNIT_Z);

	// Create the entity (the floor)
	Ogre::Entity* floor = mSceneMgr->createEntity("Floor", "FloorPlane");
	floor->setMaterialName("Examples/BumpyMetal");
	floor->setCastShadows(false); 
	Ogre::SceneNode * floorNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	floorNode->attachObject(floor);

	// Add collision detection to it
	OgreBulletCollisions::CollisionShape *floorShape;
	floorShape = new OgreBulletCollisions::StaticPlaneCollisionShape(Ogre::Vector3::UNIT_Y, 0); // (normal vector, distance)

	// A rigid body is needed for the shape
	OgreBulletDynamics::RigidBody *floorBody = new OgreBulletDynamics::RigidBody("FloorBody", mWorld);
	floorBody->setStaticShape(floorShape, 0.1, 0.8); // (shape, restitution, friction)

	// Push the created objects to the deques

	ViRus::Hittable *floorHittable = new ViRus::HitObstacle(floorBody, floorShape, floorNode);
	hitmap.add_hittable(*floorBody->getBulletObject(), *floorHittable);

	// Define the penguin mesh
	Ogre::Entity* penguin = mSceneMgr->createEntity("Penguin", "penguin.mesh");
	penguin->setCastShadows(true);
	Ogre::SceneNode *penguinNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("PenguinNode");
	penguinNode->attachObject(penguin);

	// We need the bounding box of the entity to be able to set the size of the Bullet shape
	Ogre::AxisAlignedBox penguinBoundingBox = penguin->getBoundingBox();

	// Size of the Bullet shape, a box
	Ogre::Vector3 penguinShapeSize = Ogre::Vector3::ZERO;
	penguinShapeSize = penguinBoundingBox.getSize();
	penguinShapeSize /= 2.0f; // Only the half needed
	penguinShapeSize *= 0.96f; // Bullet margin is a bit bigger so we need a smaller size

	penguinNode->scale(PENGUIN_SCALING, PENGUIN_SCALING, PENGUIN_SCALING); // The penguin is too big for us
	penguinShapeSize *= PENGUIN_SCALING; // don't forget to scale down the Bullet shape too

	penguinNode->yaw(Ogre::Degree(180));

	penguinNode->translate(0.0, 24*PENGUIN_SCALING, 0.0);

	// After that create the Bullet shape with the calculated size
	OgreBulletCollisions::BoxCollisionShape *penguinShape;
	penguinShape = new OgreBulletCollisions::BoxCollisionShape(penguinShapeSize);


	// and the Bullet rigid body
	OgreBulletDynamics::RigidBody *penguinBody = new OgreBulletDynamics::RigidBody("penguinBody", mWorld);
	Ogre::Vector3 penguinPosition = penguinNode->getPosition();
	Ogre::Quaternion penguinOrientation = penguinNode->getOrientation();
	penguinBody->setStaticShape(penguinNode, penguinShape, 0.6, 0.6, // (node, shape, restitution, friction,
	penguinPosition, penguinOrientation); // starting position, orientation)
	penguinBody->setKinematicObject(true);
	penguinBody->disableDeactivation();

	// Push the created objects to the deques

	ptr_hero = new ViRus::HitCharacter(penguinBody, penguinShape, penguinNode,  ViRus::TeamType::HERO, 40);
	ptr_hero->set_callback(TutorialApplication::hero_callback);

	hitmap.add_hittable(*penguinBody->getBulletObject(), *ptr_hero);



	// Define the penguin2 mesh
	Ogre::Entity* penguin2 = mSceneMgr->createEntity("Penguin2", "penguin.mesh");
	penguin2->setCastShadows(true);
	Ogre::SceneNode *penguin2Node = mSceneMgr->getRootSceneNode()->createChildSceneNode("Penguin2Node");
	Ogre::SceneNode *penguin2EntityNode = penguin2Node->createChildSceneNode("Penguin2Entity");
	penguin2EntityNode->attachObject(penguin2);
	penguin2EntityNode->yaw(Ogre::Degree(180));
	
	// We need the bounding box of the entity to be able to set the size of the Bullet shape
	Ogre::AxisAlignedBox penguin2BoundingBox = penguin2->getBoundingBox();

	// Size of the Bullet shape, a box
	Ogre::Vector3 penguin2ShapeSize = Ogre::Vector3::ZERO;
	penguin2ShapeSize = penguin2BoundingBox.getSize();
	penguin2ShapeSize /= 2.0f; // Only the half needed
	penguin2ShapeSize *= 0.96f; // Bullet margin is a bit bigger so we need a smaller size

	penguin2Node->scale(PENGUIN_SCALING, PENGUIN_SCALING, PENGUIN_SCALING); // The penguin2 is too big for us
	penguin2ShapeSize *= PENGUIN_SCALING; // don't forget to scale down the Bullet shape too

	penguin2Node->translate(0.0, 24*PENGUIN_SCALING, -50.0);

	// After that create the Bullet shape with the calculated size
	OgreBulletCollisions::BoxCollisionShape *penguin2Shape;
	penguin2Shape = new OgreBulletCollisions::BoxCollisionShape(penguin2ShapeSize);


	// and the Bullet rigid body
	OgreBulletDynamics::RigidBody *penguin2Body = new OgreBulletDynamics::RigidBody("penguin2Body", mWorld);
	Ogre::Vector3 penguin2Position = penguin2Node->getPosition();
	Ogre::Quaternion penguin2Orientation = penguin2Node->getOrientation();
	penguin2Body->setShape(penguin2Node, penguin2Shape, 0.1, 5.0f, 10.0f,// (node, shape, restitution, friction, mass,
		penguin2Position, penguin2Orientation); // starting position, orientation)
	penguin2Body->getBulletRigidBody()->setAngularFactor(btVector3(0, 0, 0));

	// Push the created objects to the deques

	ptr_enemy = new ViRus::HitCharAttack(penguin2Body, penguin2Shape, penguin2Node, ViRus::TeamType::ENEMY, 40, 20);
	ptr_enemy->set_callback(enemy_callback);

	hitmap.add_hittable(*penguin2Body->getBulletObject(), *ptr_enemy);
}

//-------------------------------------------------------------------------------------
void TutorialApplication::destroyScene(void)
{
	BaseApplication::destroyScene();

	hitmap.clear_all();

	delete mWorld->getDebugDrawer();
	mWorld->setDebugDrawer(nullptr);
	delete mWorld;
}
//-------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------ -
bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	bool ret = BaseApplication::frameRenderingQueued(evt);
	if (!processUnbufferedInput(evt)) return false;
	return ret;
}
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
bool TutorialApplication::processUnbufferedInput(const Ogre::FrameEvent& evt)
{
	static Ogre::Real mToggle = 0.0; // The time left until next toggle

	mToggle -= evt.timeSinceLastFrame;

	

	if (mKeyboard->isKeyDown(OIS::KC_B) && (mToggle < 0.0f) && ptr_hero)
	{
		mToggle = 0.5;

		// Create and throw a barrel if 'B' is pressed
		// Starting position of the barrel
		Ogre::SceneNode *pNode = mSceneMgr->getSceneNode("PenguinNode");
		Ogre::Vector3 from = pNode->getPosition();	//mCamera->getDerivedPosition();
		Ogre::Vector3 dir = pNode->getOrientation() * Ogre::Vector3(0,0,1);	//mCamera->getDerivedDirection();
		Ogre::Vector3 position = (from + dir.normalisedCopy() * 10.0f);

		// Create an ordinary, Ogre mesh with texture
		Ogre::Entity *barrel = mSceneMgr->createEntity(
			"Barrel" + Ogre::StringConverter::toString(mNumEntitiesInstanced), "Barrel.mesh");
		barrel->setCastShadows(true);
		Ogre::SceneNode *barrelNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		barrelNode->attachObject(barrel);

		// We need the bounding box of the entity to be able to set the size of the Bullet shape
		Ogre::AxisAlignedBox barrelBoundingBox = barrel->getBoundingBox();

		// Size of the Bullet shape, a box
		Ogre::Vector3 barrelShapeSize = Ogre::Vector3::ZERO;
		barrelShapeSize = barrelBoundingBox.getSize();
		barrelShapeSize /= 2.0f; // Only the half needed
		barrelShapeSize *= 0.25f; // Bullet margin is a bit bigger so we need a smaller size

		barrelNode->scale(Ogre::Vector3(0.25f,0.25f,0.25f));

		// After that create the Bullet shape with the calculated size
		OgreBulletCollisions::BoxCollisionShape *barrelShape;
		barrelShape = new OgreBulletCollisions::BoxCollisionShape(barrelShapeSize);

		// and the Bullet rigid body
		OgreBulletDynamics::RigidBody *barrelBody = new OgreBulletDynamics::RigidBody(
			"defaultBoxRigid" + Ogre::StringConverter::toString(mNumEntitiesInstanced), mWorld);
		barrelBody->setShape(barrelNode, barrelShape,
			0.6f, // dynamic body restitution
			0.6f, // dynamic body friction
			100.0f, // dynamic bodymass
			position, // starting position of the shape
			Ogre::Quaternion(0, 0, 0, 1)); // orientation of the shape
		barrelBody->setLinearVelocity(dir.normalisedCopy() * 100.0f); // shooting speed


		barrelBody->getBulletRigidBody()->setAngularFactor(btVector3(0, 0, 0));

		mNumEntitiesInstanced++;

		// Push the created objects to the deques

		ViRus::Hittable *barrelHittable = new ViRus::HitProjectile(barrelBody, barrelShape, barrelNode, ViRus::TeamType::HERO, 10);

		hitmap.add_hittable(*barrelBody->getBulletObject(), *barrelHittable);
	}

	static Ogre::Real mMove = 10; // The movement constant
	Ogre::Vector3 transVector = Ogre::Vector3::ZERO;

	if (mKeyboard->isKeyDown(OIS::KC_I)) // Backward
	{
		transVector.z -= mMove;
	}
	if (mKeyboard->isKeyDown(OIS::KC_K)) // Forward
	{
		transVector.z += mMove;
	}
	if (mKeyboard->isKeyDown(OIS::KC_J)) // Left
	{
		transVector.x -= mMove;
	}
	if (mKeyboard->isKeyDown(OIS::KC_L)) // Right
	{
		transVector.x += mMove;
	}
	if (ptr_hero)
		mSceneMgr->getSceneNode("PenguinNode")->translate(transVector * evt.timeSinceLastFrame);

	if (ptr_enemy)
		ptr_enemy->deltaTime(evt.timeSinceLastFrame);

	// Update Bullet Physics animation
	mWorld->stepSimulation(evt.timeSinceLastFrame);

	btDynamicsWorld * mBulletWorld = mWorld->getBulletDynamicsWorld();
	int numManifolds = mBulletWorld->getDispatcher()->getNumManifolds();
	for (int i = 0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = mBulletWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = const_cast<btCollisionObject*>(contactManifold->getBody1());

		hitmap.handle_collision(obA, obB);
	}

	hitmap.clean_queued();

	// Chase
	
	if (ptr_enemy && ptr_hero)
	{
		ptr_enemy->chase(*ptr_hero);
	}

	return true;
}
void TutorialApplication::hero_callback(ViRus::Hittable *)
{
	ptr_hero = nullptr;
}
void TutorialApplication::enemy_callback(ViRus::Hittable * h)
{
	ptr_enemy = nullptr;
}
//-------------------------------------------------------------------------------------


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        TutorialApplication app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
