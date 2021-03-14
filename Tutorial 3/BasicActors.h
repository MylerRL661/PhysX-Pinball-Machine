#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	///Plane class
	class Plane : public StaticActor
	{
	public:
		//A plane with default paramters: XZ plane centred at (0,0,0)
		Plane(PxVec3 normal=PxVec3(0.f, 1.f, 0.f), PxReal distance=0.f) 
			: StaticActor(PxTransformFromPlaneEquation(PxPlane(normal, distance)))
		{
			CreateShape(PxPlaneGeometry());
		}
	};

	///Sphere class
	class Sphere : public DynamicActor
	{
	public:
		//a sphere with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m
		// - denisty: 1kg/m^3
		Sphere(const PxTransform& pose=PxTransform(PxIdentity), PxReal radius=1.f, PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxSphereGeometry(radius), density);
		}
	};

	///Box class
	class Box : public DynamicActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		Box(const PxTransform& pose=PxTransform(PxIdentity), PxVec3 dimensions=PxVec3(.5f,.5f,.5f), PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	//static/ stationary box used for plunger
	class StaticBox : public StaticActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		StaticBox(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	//pinball area or game board compound object, all objects use the same dimensions so that everything is easy to connect and equal
	//then the individual objects actual positions are changed in the GetShape to where they need to be
	class PinballArea : public StaticActor
	{
	public:

		PinballArea(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(10.0f, 0.1f, 1.0f), PxReal density = 1.f) : StaticActor(pose)
		{
			//layout of a pinball board, showing x, y, z positions and density - flat
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y * 4, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y * 4, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x * 3, dimensions.y * 2, dimensions.z / 2), density);
			CreateShape(PxBoxGeometry(dimensions.x * 3, dimensions.y * 2, dimensions.z / 2), density);
			CreateShape(PxBoxGeometry(dimensions.x * (dimensions.y * 2), dimensions.y * 4, dimensions.z / 4), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y * 3, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 + dimensions.y / 6, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 + dimensions.y / 6, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 + dimensions.y / 4, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 + dimensions.y / 4, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 + dimensions.y / 4, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 - dimensions.y / 8, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 - dimensions.y / 8, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2 - dimensions.y / 5, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 2, dimensions.z * 3), density);
			CreateShape(PxBoxGeometry(dimensions.x / 2, dimensions.y / 4, dimensions.z * 3), density);
			//GetShape used to change the rotations of the shapes to create the play area
			GetShape(0)->setLocalPose(PxTransform(PxVec3(20.0f, 25.0f, 0.0f), PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(-20.0f, 25.0f, 0.0f), PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(0.0f, 25.0f, 40.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.0f, 25.0f, -40.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
			GetShape(4)->setLocalPose(PxTransform(PxVec3(0.0f, 23.0f, 0.0f), PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			//Plunger wall
			GetShape(5)->setLocalPose(PxTransform(PxVec3(16.0f, 25.0f, 10.0f), PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			//Bottom walls
			GetShape(6)->setLocalPose(PxTransform(PxVec3(10.0f, 25.0f, 30.0f), PxQuat(-PxHalfPi / 2 - PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(7)->setLocalPose(PxTransform(PxVec3(-10.0f, 25.0f, 30.0f), PxQuat(PxHalfPi / 2 + PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(17)->setLocalPose(PxTransform(PxVec3(-18.0f, 25.0f, 25.0f), PxQuat(PxHalfPi / 2 + PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			//Corners
			GetShape(9)->setLocalPose(PxTransform(PxVec3(13.5f, 25.0f, -35.0f), PxQuat(PxHalfPi / 2 + PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(10)->setLocalPose(PxTransform(PxVec3(-13.5f, 25.0f, -35.0f), PxQuat(-PxHalfPi / 2 - PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			//Flipper walls
			GetShape(11)->setLocalPose(PxTransform(PxVec3(9.0f, 25.0f, 25.25f), PxQuat(-PxHalfPi / 2 - PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(12)->setLocalPose(PxTransform(PxVec3(-10.0f, 25.0f, 24.25f), PxQuat(PxHalfPi / 2 + PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(13)->setLocalPose(PxTransform(PxVec3(12.0f, 25.0f, 19.25f), PxQuat(PxPi, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(14)->setLocalPose(PxTransform(PxVec3(-14.0f, 25.0f, 19.25f), PxQuat(PxPi, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			//Bars at te side on the middle
			GetShape(15)->setLocalPose(PxTransform(PxVec3(12.0f, 25.0f, 2.25f), PxQuat(-PxHalfPi / 2 - PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
			GetShape(16)->setLocalPose(PxTransform(PxVec3(-16.0f, 25.0f, 2.25f), PxQuat(PxHalfPi / 2 + PxHalfPi / 8, PxVec3(0.0f, 1.0f, 0.0f)) * PxQuat(PxHalfPi, PxVec3(1.0f, 0.0f, 0.0f))));
		}
	};

	class Capsule : public DynamicActor
	{
	public:
		Capsule(const PxTransform& pose=PxTransform(PxIdentity), PxVec2 dimensions=PxVec2(1.f,1.f), PxReal density=1.f) 
			: DynamicActor(pose)
		{
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
		}
	};

	///The ConvexMesh class
	class ConvexMesh : public DynamicActor
	{
	public:
		//constructor
		ConvexMesh(const std::vector<PxVec3>& verts, const PxTransform& pose=PxTransform(PxIdentity), PxReal density=1.f)
			: DynamicActor(pose)
		{
			PxConvexMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
			mesh_desc.vertexLimit = 256;

			CreateShape(PxConvexMeshGeometry(CookMesh(mesh_desc)), density);
		}

		//mesh cooking (preparation)
		PxConvexMesh* CookMesh(const PxConvexMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if(!GetCooking()->cookConvexMesh(mesh_desc, stream))
				throw new Exception("ConvexMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createConvexMesh(input);
		}
	};

	///The TriangleMesh class
	class TriangleMesh : public StaticActor
	{
	public:
		//constructor
		TriangleMesh(const std::vector<PxVec3>& verts, const std::vector<PxU32>& trigs, const PxTransform& pose=PxTransform(PxIdentity))
			: StaticActor(pose)
		{
			PxTriangleMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.triangles.count = (PxU32)trigs.size();
			mesh_desc.triangles.stride = 3*sizeof(PxU32);
			mesh_desc.triangles.data = &trigs.front();

			CreateShape(PxTriangleMeshGeometry(CookMesh(mesh_desc)));
		}

		//mesh cooking (preparation)
		PxTriangleMesh* CookMesh(const PxTriangleMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if(!GetCooking()->cookTriangleMesh(mesh_desc, stream))
				throw new Exception("TriangleMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createTriangleMesh(input);
		}
	};

	//Distance joint with the springs switched on
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = (PxJoint*)PxDistanceJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
			((PxDistanceJoint*)joint)->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
			Damping(20.f);
			Stiffness(20.f);
		}

		void Stiffness(PxReal value)
		{
			((PxDistanceJoint*)joint)->setStiffness(value);
		}

		PxReal Stiffness()
		{
			return ((PxDistanceJoint*)joint)->getStiffness();		
		}

		void Damping(PxReal value)
		{
			((PxDistanceJoint*)joint)->setDamping(value);
		}

		PxReal Damping()
		{
			return ((PxDistanceJoint*)joint)->getDamping();
		}
	};

	///Revolute Joint
	class RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = PxRevoluteJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION,true);
		}

		void DriveVelocity(PxReal value)
		{
			//wake up the attached actors
			PxRigidDynamic *actor_0, *actor_1;
			((PxRevoluteJoint*)joint)->getActors((PxRigidActor*&)actor_0, (PxRigidActor*&)actor_1);
			if (actor_0)
			{
				if (actor_0->isSleeping())
					actor_0->wakeUp();
			}
			if (actor_1)
			{
				if (actor_1->isSleeping())
					actor_1->wakeUp();
			}
			((PxRevoluteJoint*)joint)->setDriveVelocity(value);
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
		}

		PxReal DriveVelocity()
		{
			return ((PxRevoluteJoint*)joint)->getDriveVelocity();
		}

		void SetLimits(PxReal lower, PxReal upper)
		{
			((PxRevoluteJoint*)joint)->setLimit(PxJointAngularLimitPair(lower, upper));
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
		}
	};

	

	//moved to where the other actors and joints are to organise the code a bit
	//pyramid vertices for flippers			
	static PxVec3 pyramid_verts[] = { PxVec3(0.5,4,0.25),PxVec3(-0.5,4,0.25),PxVec3(0.5,4,-0.25),PxVec3(-0.5,4,-0.25), PxVec3(1,0,0.5), PxVec3(-1,0,0.5), PxVec3(-1,0,-0.5), PxVec3(1,0,-0.5) };
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0

	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = { 1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1 };

	//vertices for flippers???
	static PxVec3 flap_verts[] = { PxVec3(1.25,4,0.25),PxVec3(-1.25,4,0.25),PxVec3(1.25,4,-0.25),PxVec3(-1.25,4,-0.25), PxVec3(1.25,0,0.25), PxVec3(-1.25,0,0.25), PxVec3(-1.25,0,-0.25), PxVec3(1.25,0,-0.25) };

	//used for flippers. Both moved from MyPhysicsEngine.h
	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose = PxTransform(PxIdentity), PxReal side = PxReal(0.5f), PxReal density = 1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose = PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs), end(pyramid_trigs)), pose)
		{
		}
	};
}