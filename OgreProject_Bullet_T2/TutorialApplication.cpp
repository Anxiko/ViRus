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
	floor->setCastShadows(false); mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(floor);

	// Add collision detection to it
	OgreBulletCollisions::CollisionShape *floorShape;
	floorShape = new OgreBulletCollisions::StaticPlaneCollisionShape(Ogre::Vector3::UNIT_Y, 0); // (normal vector, distance)

	// A rigid body is needed for the shape
	OgreBulletDynamics::RigidBody *floorBody = new OgreBulletDynamics::RigidBody("FloorBody", mWorld);
	floorBody->setStaticShape(floorShape, 0.1, 0.8); // (shape, restitution, friction)

	// Push the created objects to the deques
	mShapes.push_back(floorShape);
	mBodies.push_back(floorBody);

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

	penguinNode->scale(0.25f, 0.25f, 0.25f); // The penguin is too big for us
	penguinShapeSize *= 0.25f; // don't forget to scale down the Bullet shape too

	penguinNode->translate(0.0, 6.0, 0.0);

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
	mShapes.push_back(penguinShape);
	mBodies.push_back(penguinBody);

	penguinBulletObject = penguinBody->getBulletObject();
}

//-------------------------------------------------------------------------------------
void TutorialApplication::destroyScene(void)
{
	BaseApplication::destroyScene();

	for (auto ptr_rigid : mBodies)
		delete ptr_rigid;

	mBodies.clear();

	for (auto ptr_collisionShape : mShapes)
		delete ptr_collisionShape;

	mShapes.clear();

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

	if (mKeyboard->isKeyDown(OIS::KC_B) && (mToggle < 0.0f))
	{
		mToggle = 0.5;

		// Create and throw a barrel if 'B' is pressed

		// Starting position of the barrel
		Ogre::Vector3 from = mCamera->getDerivedPosition();
		Ogre::Vector3 dir = mCamera->getDerivedDirection();
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
		barrelShapeSize *= 0.96f; // Bullet margin is a bit bigger so we need a smaller size

		// After that create the Bullet shape with the calculated size
		OgreBulletCollisions::BoxCollisionShape *barrelShape;
		barrelShape = new OgreBulletCollisions::BoxCollisionShape(barrelShapeSize);

		// and the Bullet rigid body
		OgreBulletDynamics::RigidBody *barrelBody = new OgreBulletDynamics::RigidBody(
			"defaultBoxRigid" + Ogre::StringConverter::toString(mNumEntitiesInstanced), mWorld);
		barrelBody->setShape(barrelNode, barrelShape,
			0.6f, // dynamic body restitution
			0.6f, // dynamic body friction
			1.0f, // dynamic bodymass
			position, // starting position of the shape
			Ogre::Quaternion(0, 0, 0, 1)); // orientation of the shape
		barrelBody->setLinearVelocity(dir.normalisedCopy() * 7.0f); // shooting speed

		mNumEntitiesInstanced++;

		// Push the created objects to the deques
		mShapes.push_back(barrelShape);
		mBodies.push_back(barrelBody);
	}

	static Ogre::Real mMove = 250; // The movement constant
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

	mSceneMgr->getSceneNode("PenguinNode")->translate(transVector * evt.timeSinceLastFrame);

	// Update Bullet Physics animation
	mWorld->stepSimulation(evt.timeSinceLastFrame);

	static bool penguinHitLastFrame = false;
	bool penguinHitThisFrame = false;

	btDynamicsWorld * mBulletWorld = mWorld->getBulletDynamicsWorld();
	int numManifolds = mBulletWorld->getDispatcher()->getNumManifolds();
	for (int i = 0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = mBulletWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = const_cast<btCollisionObject*>(contactManifold->getBody1());

		if ((obA == penguinBulletObject) || (obB == penguinBulletObject))
		{
			penguinHitThisFrame = true;
			break;
		}
	}

	if (!penguinHitLastFrame && penguinHitThisFrame)
		mSceneMgr->getSceneNode("PenguinNode")->pitch(Ogre::Degree(90.0));

	penguinHitLastFrame = penguinHitThisFrame;

	return true;
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
