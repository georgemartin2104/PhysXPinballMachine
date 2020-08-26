#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	// Defines triangle bounds for the feet of the machine
	static const PxVec3 foot_verts[] = { PxVec3(0,0,0), PxVec3(1,0,0), PxVec3(0,.18,0), PxVec3(1,.18,0), PxVec3(0,0,-1), PxVec3(1,0,-1) };


	//Distance joint with the springs switched on - used for the plunger
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
			Damping(1.f);
			Stiffness(1.f);
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

	///Revolute Joint - used for the flippers
	class RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = PxRevoluteJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
		}

		void DriveVelocity(PxReal value)
		{
			//wake up the attached actors
			PxRigidDynamic* actor_0, * actor_1;
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

	// Creates the pinball case
	class PinballCase : public StaticActor
	{
	public:
		PinballCase(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 1.f)
			: StaticActor(pose)
		{
			// Materials for the case
			PxMaterial* backMaterial;
			PxMaterial* wallMaterial;

			backMaterial = CreateMaterial(0.f, 0.f, 0.f);
			wallMaterial = CreateMaterial(2.f, 2.f, 0.5f);


			// Floor of machine
			CreateShape(PxBoxGeometry(PxVec3(5.f, 11.f, .2f)), density);
			// Left and right sides of machine
			CreateShape(PxBoxGeometry(PxVec3(.2f, 10.6f, .8f)), density);
			CreateShape(PxBoxGeometry(PxVec3(.2f, 12.8f, .8f)), density);
			// Top and bottom of machine
			CreateShape(PxBoxGeometry(PxVec3(5.f, .2f, .8f)), density);
			CreateShape(PxBoxGeometry(PxVec3(4.3f, .2f, .8f)), density);
			// Long legs
			CreateShape(PxBoxGeometry(PxVec3(.5f, .5f, 2.5f)), density);
			CreateShape(PxBoxGeometry(PxVec3(.5f, .5f, 2.5f)), density);
			// Short Legs
			CreateShape(PxBoxGeometry(PxVec3(.5f, .5f, .8f)), density);
			CreateShape(PxBoxGeometry(PxVec3(.5f, .5f, .8f)), density);

			// Leg Triangles
			// Describe triangle mesh
			PxConvexMeshDesc footDesc;
			footDesc.points.count = 6;
			footDesc.points.stride = sizeof(PxVec3);
			footDesc.points.data = foot_verts;
			footDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

			// Cook mesh
			PxDefaultMemoryOutputStream buf;
			if (!GetCooking()->cookConvexMesh(footDesc, buf))
				throw new Exception("ConvexMesh::CookMesh, cooking failed.");
			PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
			PxConvexMesh* convexMesh = GetPhysics()->createConvexMesh(input);
			
			// Create feet
			CreateShape(PxConvexMeshGeometry(convexMesh), PxReal(1.f));
			CreateShape(PxConvexMeshGeometry(convexMesh), PxReal(1.f));
			CreateShape(PxConvexMeshGeometry(convexMesh), PxReal(1.f));
			CreateShape(PxConvexMeshGeometry(convexMesh), PxReal(1.f));

			// Create interior shape near plunger
			CreateShape(PxBoxGeometry(PxVec3(.2f, 7.0f, .8f)), density);

			// Create slope
			CreateShape(PxBoxGeometry(PxVec3(4.2f, .2f, .4f)), density);

			// Create top corners
			CreateShape(PxBoxGeometry(PxVec3(1.0f, .2f, .4f)), density);
			CreateShape(PxBoxGeometry(PxVec3(1.0f, .2f, .4f)), density);

			// Create top and bottom plunger enclosure
			CreateShape(PxBoxGeometry(PxVec3(.5f, 2.2f, .2f)), density);
			CreateShape(PxBoxGeometry(PxVec3(.5f, 2.2f, .2f)), density);

			// Create left plunger enclosure
			CreateShape(PxBoxGeometry(PxVec3(.2f, 2.2f, .8f)), density);

			// Create rear plunger enclosure
			CreateShape(PxBoxGeometry(PxVec3(.6f, .4f, .8f)), density);

			// Create slopes near flippers
			CreateShape(PxBoxGeometry(PxVec3(1.f, .2f, .4f)), density);
			CreateShape(PxBoxGeometry(PxVec3(1.f, .2f, .4f)), density);

			// Position parts of compound shape and allocate materials
			GetShape(0)->setLocalPose(PxTransform(PxVec3(0.f, 0.f, 0.f))); // Floor of case
			GetShape(0)->setMaterials(&backMaterial, 1);
			GetShape(1)->setLocalPose(PxTransform(PxVec3(-4.8f, 0.f, 1.f))); // Left side of machine
			GetShape(1)->setMaterials(&wallMaterial, 1);
			GetShape(2)->setLocalPose(PxTransform(PxVec3(4.8f, -2.2f, 1.f))); // Right side of machine
			GetShape(2)->setMaterials(&wallMaterial, 1);
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.f, 10.8f, 1.f))); // Top of machine
			GetShape(3)->setMaterials(&wallMaterial, 1);
			GetShape(4)->setLocalPose(PxTransform(PxVec3(-.7f, -10.8f, 1.f))); // Bottom of machine
			GetShape(4)->setMaterials(&wallMaterial, 1);
			GetShape(5)->setLocalPose(PxTransform(PxVec3(-3.5f, 9.f, -2.7f))); // Rear legs
			GetShape(5)->setMaterials(&wallMaterial, 1);
			GetShape(6)->setLocalPose(PxTransform(PxVec3(3.5f, 9.f, -2.7f)));
			GetShape(6)->setMaterials(&wallMaterial, 1);
			GetShape(7)->setLocalPose(PxTransform(PxVec3(-3.5f, -10.f, -1.f))); // Front legs
			GetShape(7)->setMaterials(&wallMaterial, 1);
			GetShape(8)->setLocalPose(PxTransform(PxVec3(3.5f, -10.f, -1.f)));
			GetShape(8)->setMaterials(&wallMaterial, 1);
			GetShape(9)->setLocalPose(PxTransform(PxVec3(3.0f, -9.5f, -1.8f), PxQuat(-1.5708f, PxVec3(1.0f, .0f, .0f)))); // Triangles for the 'feet' of the legs
			GetShape(9)->setMaterials(&wallMaterial, 1);
			GetShape(10)->setLocalPose(PxTransform(PxVec3(-4.0f, -9.5f, -1.8f), PxQuat(-1.5708f, PxVec3(1.0f, .0f, .0f))));
			GetShape(10)->setMaterials(&wallMaterial, 1);
			GetShape(11)->setLocalPose(PxTransform(PxVec3(3.0f, 9.5f, -5.2f), PxQuat(-1.5708f, PxVec3(1.0f, .0f, .0f))));
			GetShape(11)->setMaterials(&wallMaterial, 1);
			GetShape(12)->setLocalPose(PxTransform(PxVec3(-4.0f, 9.5f, -5.2f), PxQuat(-1.5708f, PxVec3(1.0f, .0f, .0f))));
			GetShape(12)->setMaterials(&wallMaterial, 1);
			GetShape(13)->setLocalPose(PxTransform(PxVec3(3.3f, -1.f, 1.f))); // Interior shape near plunger
			GetShape(13)->setMaterials(&wallMaterial, 1);
			GetShape(14)->setLocalPose(PxTransform(PxVec3(-0.6f, -9.5f, 1.f), PxQuat(-0.261799f, PxVec3(.0f, .0f, 1.0f)))); // Slope
			GetShape(14)->setMaterials(&wallMaterial, 1);
			GetShape(15)->setLocalPose(PxTransform(PxVec3(3.8f, 10.0f, 1.f), PxQuat(-0.261799f, PxVec3(.0f, .0f, 1.0f)))); // Top Corners
			GetShape(15)->setMaterials(&wallMaterial, 1);
			GetShape(16)->setLocalPose(PxTransform(PxVec3(-3.8f, 10.0f, 1.f), PxQuat(0.261799f, PxVec3(.0f, .0f, 1.0f))));
			GetShape(16)->setMaterials(&wallMaterial, 1);
			GetShape(17)->setLocalPose(PxTransform(PxVec3(4.1f, -12.8f, 1.6f))); // Plunger enclosure
			GetShape(17)->setMaterials(&wallMaterial, 1);
			GetShape(18)->setLocalPose(PxTransform(PxVec3(4.1f, -12.8f, 0.f)));
			GetShape(18)->setMaterials(&wallMaterial, 1);
			GetShape(19)->setLocalPose(PxTransform(PxVec3(3.5f, -12.8f, 0.8f)));
			GetShape(19)->setMaterials(&wallMaterial, 1);
			GetShape(20)->setLocalPose(PxTransform(PxVec3(4.1f, -15.f, 0.8f)));
			GetShape(20)->setMaterials(&wallMaterial, 1);
			GetShape(21)->setLocalPose(PxTransform(PxVec3(-3.5f, -5.f, 1.f), PxQuat(-0.261799f, PxVec3(.0f, .0f, 1.0f)))); // Flipper slopes
			GetShape(21)->setMaterials(&wallMaterial, 1);
			GetShape(22)->setLocalPose(PxTransform(PxVec3(2.f, -5.f, 1.f), PxQuat(0.261799f, PxVec3(.0f, .0f, 1.0f))));
			GetShape(22)->setMaterials(&wallMaterial, 1);
		}
	};

	///Sphere class
	class Pinball : public DynamicActor
	{
	public:
		
		// Premade sphere

		Pinball(const PxTransform& pose = PxTransform(PxIdentity), PxReal radius = .4f, PxReal density = 1.f)
			: DynamicActor(pose)
		{
			CreateShape(PxSphereGeometry(radius), density);
		}
	};

	//Plunger class
	class Plunger
	{
		vector<DistanceJoint*> springs;
		Box* base, * top;
	public:
		Plunger(const PxTransform& pose = PxTransform(PxIdentity), PxReal stiffness = 100.f, PxReal damping = 10.f)
		{
			const PxVec3 dimensions = PxVec3(.4f, .4f, .4f);

			base = new Box(pose, PxVec3(dimensions.x, .1f, dimensions.z));
			top = new Box(pose, PxVec3(dimensions.x, 2.f, dimensions.z));
			// Base is set to kinematic as it is not used
			base->SetKinematic(true);
			springs.resize(4);
			springs[0] = new DistanceJoint(base, PxTransform(PxVec3(dimensions.x, 3.f, dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -.1f, dimensions.z)));
			springs[1] = new DistanceJoint(base, PxTransform(PxVec3(dimensions.x, 3.f, -dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -.1f, -dimensions.z)));
			springs[2] = new DistanceJoint(base, PxTransform(PxVec3(-dimensions.x, 3.f, dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -.1f, dimensions.z)));
			springs[3] = new DistanceJoint(base, PxTransform(PxVec3(-dimensions.x, 3.f, -dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -.1f, -dimensions.z)));

			springs[0]->Get()->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}
		// Adds plunger to scene
		void AddToScene(Scene* scene)
		{
			((PxRigidDynamic*)top->Get())->setRigidDynamicLockFlags(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z | PxRigidDynamicLockFlag::eLOCK_ANGULAR_X | PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y | PxRigidDynamicLockFlag::eLOCK_LINEAR_X);
			base->GetShape(0)->setLocalPose(PxTransform(PxVec3(-2.f, -3.f, -5.f)));
			scene->Add(base);
			scene->Add(top);
		}
		// Called when plunger key is pressed
		void use()
		{
			((PxRigidDynamic*)top->Get())->addForce((PxVec3(0, 0, 1) * 2000));
			((PxRigidDynamic*)top->Get())->addForce((PxVec3(0, -1, 0) * 20));
		}
		// Deconstructor
		~Plunger()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};
	
	//Flipper Class
	class Flipper : public DynamicActor
	{
		RevoluteJoint* revJoint;

	public:
		Flipper(bool isLeftFlipper, const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			// Flipper materials
			PxMaterial* flipperMaterial;

			flipperMaterial = CreateMaterial(.0f, .0f, 0.5f);

			// Flipper shape
			CreateShape(PxBoxGeometry(PxVec3(.4f, .6f, .1f)), density);
			GetShape(0)->setMaterials(&flipperMaterial, 1);

			// Determines direction based on if flipper is on left or right side
			if (isLeftFlipper)
			{
				revJoint = new RevoluteJoint(nullptr, PxTransform(pose.p, pose.q * PxQuat(PxPi / 2, PxVec3(0, 0, 1))), this, PxTransform(PxVec3(0, .4f, 0)));
				revJoint->SetLimits(-PxPi / 5, PxPi / 3);

				((PxRigidActor*)this->Get())->setName("leftFlipper");
			}

			if (!isLeftFlipper)
			{
				revJoint = new RevoluteJoint(nullptr, PxTransform(pose.p, pose.q * PxQuat(PxPi / 2, PxVec3(0, 0, 1))), this, PxTransform(PxVec3(0, -.4f, 0)));
				revJoint->SetLimits(-PxPi / 3, PxPi / 5);

				((PxRigidActor*)this->Get())->setName("rightFlipper");
			}



		}

		// Deconstructor
		~Flipper()
		{
			if (revJoint != nullptr)
			{
				delete revJoint;
			}
		}

		RevoluteJoint* GetJoint()
		{
			return revJoint;
		}
	};

	// Bumper class
	class Bumper : public StaticActor
	{
	public:
		Bumper(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 1.f)
			: StaticActor(pose)
		{
			// Bumper material
			PxMaterial* bumperMaterial;

			bumperMaterial = CreateMaterial(0.f, 0.f, 3.f);

			// Bumper shape
			CreateShape(PxBoxGeometry(PxVec3(.2f, .4f, .2f)), density);
			CreateShape(PxSphereGeometry(.6f), density);

			// Bumper positioning and material allocation
			GetShape(0)->setLocalPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.f, .3f, 0.f)));
			GetShape(1)->setMaterials(&bumperMaterial, 1);
		}
	};

	class Spinner : public DynamicActor
	{
	public:
		Spinner(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			// Spinner Material
			PxMaterial* spinnerMaterial;

			spinnerMaterial = CreateMaterial(0.f, 0.f, 1.5f);

			// Spinner Shape
			CreateShape(PxBoxGeometry(PxVec3(1.8f, .2f, .4f)), density);

			// Spinner positioning and material allocation
			GetShape(0)->setLocalPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
			GetShape(0)->setMaterials(&spinnerMaterial, 1);
		}
	};
}