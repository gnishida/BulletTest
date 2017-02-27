#include "BulletSim.h"
#include "Utils.h"
#include <iostream>

BulletSim::BulletSim() {
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -9.8, 0));

	init();
}

BulletSim::~BulletSim() {
	clear();

	delete dynamicsWorld;
	delete solver;
	delete overlappingPairCache;
	delete dispatcher;
	delete collisionConfiguration;
}

void BulletSim::draw(QPainter& painter) {
	for (int i = 0; i < dynamicsWorld->getNumCollisionObjects(); i++) {
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;
		if (body && body->getMotionState()) {
			body->getMotionState()->getWorldTransform(trans);

			float x = trans.getOrigin().getX() * 100;
			float y = 800 - trans.getOrigin().getY() * 100;
			btQuaternion qt = trans.getRotation();

			painter.save();
			painter.translate(x, y);
			painter.rotate(-atan2f(2 * qt.z() * qt.w(), (1 - 2 * qt.z() * qt.z())) / 3.141592 * 180);

			if (obj->getCollisionShape()->getShapeType() == BOX_SHAPE_PROXYTYPE) {
				btBoxShape* shape = static_cast<btBoxShape*>(obj->getCollisionShape());
				btVector3 size = shape->getHalfExtentsWithMargin();
				painter.drawRect(-size.x() * 100, -size.y() * 100, size.x() * 200, size.y() * 200);
			}

			painter.restore();
		}
		else {
			trans = obj->getWorldTransform();
		}
		//printf("world pos object %d = %f,%f,%f\n", i, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));

		
	}
}

void BulletSim::init() {
	clear();

	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));

		collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(4, -4, 0));

		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		dynamicsWorld->addRigidBody(body);
	}

	for (int i = 0; i < 100; i++) {
		//create a dynamic rigidbody

		//btCollisionShape* colShape = new btSphereShape(btScalar(0.2));
		btCollisionShape* colShape = new btBoxShape(btVector3(0.2, 0.2, 0.2));
		collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(utils::genRand(1, 7), utils::genRand(2, 7), 0));

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		// move/rotate only in X-Y plane
		body->setLinearFactor(btVector3(1, 1, 0));
		body->setAngularFactor(btVector3(0, 0, 1));

		dynamicsWorld->addRigidBody(body);
	}
}

void BulletSim::clear() {
	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	for (int i = 0; i < collisionShapes.size(); i++) {
		btCollisionShape* shape = collisionShapes[i];
		collisionShapes[i] = 0;
		delete shape;
	}
}

void BulletSim::stepSimulation(float timeStep) {
	dynamicsWorld->stepSimulation(timeStep, 10);
}