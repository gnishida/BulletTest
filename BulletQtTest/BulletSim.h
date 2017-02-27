#pragma once

#include "btBulletDynamicsCommon.h"
#include <QPainter>

class BulletSim {
private:
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

public:
	BulletSim();
	~BulletSim();

	void draw(QPainter& painter);
	void init();
	void clear();
	void stepSimulation(float timeStep);
};

