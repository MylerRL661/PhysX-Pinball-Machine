#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>
#include "VisualDebugger.h"

//CODE COMMENT REPORT 300 WORDS
/*
	The first part of the pinball implementation was building the board.
	This was done by setting a default dimensions then changing the shape and rotation using the “GetShape” function.
	This just holds the walls of the board.
	This is spawned into the world and defined with a name, colour, etc. All the other actor and objects are initialised and put into the game and renamed one by one.
	A score and lives are also initialised as 0 and 3. The ball was made smaller and the material changed to make it like a pinball.
	A pyramid object was created that was used for the flippers, a rigidbody was added to them as well so that they can hit and react to the pinball.
	Capsules with revolute joints from workshop 3 were added so that they would stay in an upright position.
	These were turned into triggers that add different score values as they are hit, by setting the triggers up in the “onTrigger” function and using bools to create if statements that do different things such as add score or respawn the ball once it goes out of bounds.
	A trampoline was added and a new function was added to it to apply force on a button press (key3) to fire the ball into the play area. A custom update function holds all the custom events which use the triggers.
	The pinball is reset by using the removeActor function and adding a new version of the original pinball at the start position. The resetPinball and all trigger functionality are original as well as the plunger function, buttons, score setup and score from capsules.
	The code should work using the normal setup in Visual Studio 2019 as shown through a workshop tutorial.
	Citation: Gameworksdocs.nvidia.com. 2020. NVIDIA Physx SDK 4.0 Documentation — NVIDIA Physx SDK 4.0 Documentation. [online] Available at: <https://gameworksdocs.nvidia.com/PhysX/4.0/documentation/PhysXGuide/Index.html> [Accessed 30 April 2020].Citation: Gameworksdocs.nvidia.com. 2020. NVIDIA Physx SDK 4.0 Documentation — NVIDIA Physx SDK 4.0 Documentation. [online] Available at: <https://gameworksdocs.nvidia.com/PhysX/4.0/documentation/PhysXGuide/Index.html> [Accessed 30 April 2020].
*/

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f), PxVec3(4.f / 255.f,117.f / 255.f,111.f / 1.f) };

	//int for score
	static int Score = 0;

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		//added a static box to bottom to ensure it stays still and is implanted 
		vector<DistanceJoint*> springs;
		Box* top;
		StaticBox* bottom;

	public:
		Trampoline(const PxTransform& pose = PxTransform(PxIdentity), const PxVec3& dimensions = PxVec3(1.f, 1.f, 1.f), PxReal stiffness = 1.f, PxReal damping = 1.f)
		{
			PxReal thickness = .1f;
			bottom = new StaticBox(PxTransform(pose.p + PxVec3(.0f, .0f, .0f), pose.q), PxVec3(dimensions.x, thickness, dimensions.z));
			top = new Box(PxTransform(pose.p + PxVec3(.0f, .0f, .0f), pose.q), PxVec3(dimensions.x, thickness, dimensions.z));

			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x, thickness, dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -dimensions.y, dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x, thickness, -dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -dimensions.y, -dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x, thickness, dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -dimensions.y, dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x, thickness, -dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -dimensions.y, -dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		//force added then plunger strikes, propels the ball forwards when pressed
		void PlungerForce(PxReal force) {
			((PxRigidDynamic*)top->Get())->addForce(PxVec3(0.f, (force * 200)/ 2, -force * 200.f));
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2),
			ACTOR3		= (1 << 3),
			ACTOR4		= (1 << 4)
			//add more if you need
		};
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;
		//booleans for scoring/ multiple scores
		bool Score; //used for score but also for one of the capsules
		bool Score1; //used for a different capsule
		bool Score2; //used for another another capsule
		//boolean to check that the plunger is already been hit
		bool plungerIsPulled;
		//boolean checks if pinball is out
		bool outOfBounds;

		//MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						//cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						//trigger = true;
						//string to be able to work triggers againest other triggers
						string triggerName = std::string(pairs[i].triggerActor->getName());
						string otherName = std::string(pairs[i].otherActor->getName());

						//if ball falls out of bounds then trigger gameOver which resets balls an stop score
						if (triggerName == "OutOfBounds")
							if(otherName == "Pinball") {
								outOfBounds = true;
								plungerIsPulled = false;
							}
								
						//when hit the score starts going up until out of bounds is hit
						if (triggerName == "plungerPulled")
							if (otherName == "Pinball")
								plungerIsPulled = true;

						//when hit the score goes up by a certain amount based on capsule e.g. 1000
						if (triggerName == "Score")
							if (otherName == "Pinball")
								Score = true;

						//when hit the score goes up by a certain amount based on capsule e.g. 100
						if (triggerName == "Score1")
							if (otherName == "Pinball")
								Score1 = true;

						//when hit the score goes up by a certain amount based on capsule e.g. 250
						if (triggerName == "Score2")
							if (otherName == "Pinball")
								Score2 = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						//cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						//trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			std::cout << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//switch statement to check contact with actors
				switch (pairs[i].shapes[0]->getSimulationFilterData().word0)
				{
				case FilterGroup::ACTOR0:
			

					break;
				case FilterGroup::ACTOR1:
		

					break;
				case FilterGroup::ACTOR2:
			

					break;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
		//pairFlags |= PxPairFlag::eCCD_LINEAR;
		pairFlags = PxPairFlag::eSOLVE_CONTACT;
		pairFlags |= PxPairFlag::eDETECT_DISCRETE_CONTACT;
		pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		//actors and objects intialisation
		Plane* plane;
		Box* box, * box2;
		//trigger for out of bounds and plungerPulled
		StaticBox* boxTrigger1, * boxTrigger2;
		PinballArea* pinballBoard;
		Sphere* pinball, * pinball2;
		Capsule* capsule1, * capsule2, * capsule3, * capsule4;
		Pyramid* flipperLeft, * flipperRight;
		//joint initialisation
		RevoluteJoint* flipperLeftJoint, * flipperRightJoint, * capsuleJoint1, * capsuleJoint2, * capsuleJoint3, * capsuleJoint4;
		MySimulationEventCallback* my_callback;
		//trampoline initialisation for plunger and bumpers
		Trampoline* mainPlunger;
		
	public:

		//live counter for game
		int Lives = 3;

		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

		///A custom scene class
		void SetVisualisation()
		{
			//visualisation for hidden elements
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eBODY_LIN_VELOCITY, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eBODY_AXES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);

		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			//world floor
			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			Add(plane);

			//pinball board - setting up and position
			pinballBoard = new PinballArea(PxTransform(PxVec3(0.0f, 0.5f, 0.0f), PxQuat(PxHalfPi / 4, PxVec3(1.0f, 0.f, 0.f))), PxVec3(1.0f, 10.0f, 1.0f));
			pinballBoard->Color(color_palette[0]);
			pinballBoard->Color(PxVec3(0.0f, 0.0f, 0.0f), 4);
			pinballBoard->Name("PinballBoard");
			pinballBoard->Material(CreateMaterial(0.0f, 0.0f, 0.5f));			
			Add(pinballBoard);

			//pinball - setup and position, rigidbody for collision
			pinball = new Sphere(PxTransform(PxVec3(18.0f, 18.0f, 23.0f)));
			pinball->Name("Pinball");
			pinball->Color(color_palette[3]);
			pinball->Material(CreateMaterial(0.78f, 0.27f, 0.39f));
			((PxRigidBody*)pinball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(pinball);

			//flippers - setup and position for just the actor, rigidbody for collision
			flipperLeft = new Pyramid(PxTransform(PxVec3(0.0f, 5.0f, 5.0f)));
			flipperLeft->Color(color_palette[4]);
			flipperLeft->Name("FlipperLeft");
			flipperLeft->Material(CreateMaterial(0.78f, 0.27f, 0.39f));
			((PxRigidBody*)flipperLeft->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(flipperLeft);

			flipperRight = new Pyramid(PxTransform(PxVec3(0.0f, 5.0f, 10.0f)));
			flipperRight->Color(color_palette[4]);
			flipperRight->Name("FlipperRight");
			flipperRight->Material(CreateMaterial(0.78f, 0.27f, 0.39f));
			((PxRigidBody*)flipperRight->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(flipperRight);

			//joint setup, using revolute joints
			//right flipper
			flipperRightJoint = new RevoluteJoint(NULL, PxTransform(PxVec3(5.5f, 13.5f, 35.0f), PxQuat(PxPi / 2, PxVec3(0.f, -1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, 1.f))), flipperLeft, PxTransform(PxVec3(0.f, 0.0f, 0.f)));
			flipperRightJoint->SetLimits(PxPi / 4, (PxPi / 2 + PxPi / 8));

			//left flipper
			flipperLeftJoint = new RevoluteJoint(NULL, PxTransform(PxVec3(-5.5f, 13.5f, 35.0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, -1.f))), flipperRight, PxTransform(PxVec3(0.f, 0.0f, 0.f)));
			flipperLeftJoint->SetLimits(PxPi / 4, (PxPi / 2 + PxPi / 8));

			//capsules - add score for player
			capsule1 = new Capsule(PxTransform(PxVec3(0.0f, 5.0f, 10.0f)));
			capsule1->Color(color_palette[1]);
			capsule1->Name("Score");
			capsule1->SetTrigger(true);
			Add(capsule1);

			capsule2 = new Capsule(PxTransform(PxVec3(0.0f, 5.0f, 10.0f)));
			capsule2->Color(color_palette[4]);
			capsule2->Name("Score2");
			capsule2->SetTrigger(true);
			Add(capsule2);

			capsule3 = new Capsule(PxTransform(PxVec3(0.0f, 5.0f, 10.0f)));
			capsule3->Color(color_palette[4]);
			capsule3->Name("Score2");
			capsule3->SetTrigger(true);
			Add(capsule3);

			capsule4 = new Capsule(PxTransform(PxVec3(0.0f, 5.0f, 10.0f)));
			capsule4->Color(color_palette[5]);
			capsule4->Name("Score1");
			capsule4->SetTrigger(true);
			Add(capsule4);

			//makes the capsules have revolute joints so that they can stay upright in the same position
			capsuleJoint1 = new RevoluteJoint(NULL, PxTransform(PxVec3(0.0f, 30.0f, -5.0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, -1.f))), capsule1, PxTransform(PxVec3(0.f, 0.0f, 0.f)));
			capsuleJoint2 = new RevoluteJoint(NULL, PxTransform(PxVec3(5.0f, 34.0f, -15.0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, -1.f))), capsule2, PxTransform(PxVec3(0.f, 0.0f, 0.f)));
			capsuleJoint3 = new RevoluteJoint(NULL, PxTransform(PxVec3(-5.0f, 34.0f, -15.0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, -1.f))), capsule3, PxTransform(PxVec3(0.f, 0.0f, 0.f)));
			capsuleJoint4 = new RevoluteJoint(NULL, PxTransform(PxVec3(0.0f, 20.0f, 20.0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, -1.f))), capsule4, PxTransform(PxVec3(0.f, 0.0f, 0.f)));

			//main plunger
			mainPlunger = new Trampoline(PxTransform(PxVec3(18.0f, 15.0f, 30.0f), PxQuat(PxPi, PxVec3(.0f, .0f, 1.0f)) * PxQuat(PxHalfPi + PxHalfPi / 4, PxVec3(-1.0f, 0.0f, 0.0f))), PxVec3(1.0f, 4.0f, 1.0f), 100.0f, 25.0f);
			mainPlunger->AddToScene(this);

			//trigger for when the ball is out of the machine, i.e. life lost
			boxTrigger1 = new StaticBox(PxTransform(PxVec3(0.0f, 9.0f, 45.0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, -1.f))), PxVec3(2.0f, 2.0f, 12.0f));
			boxTrigger1->Name("OutOfBounds");
			Add(boxTrigger1);
			boxTrigger1->SetTrigger(true);
			boxTrigger1->GetShape(0)->setFlag(PxShapeFlag::eVISUALIZATION, false);

			//trigger to check that the plunger is pulled and the score can be incremented
			boxTrigger2 = new StaticBox(PxTransform(PxVec3(18.0f, 34.0f, -17.0f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f)) * PxQuat(PxHalfPi - PxHalfPi / 4, PxVec3(0.0f, 0.f, -1.f))), PxVec3(2.0f, 2.0f, 1.0f));
			Add(boxTrigger2);
			boxTrigger2->Name("plungerPulled");
			boxTrigger2->SetTrigger(true);
			boxTrigger2->GetShape(0)->setFlag(PxShapeFlag::eVISUALIZATION, false);

			//set collision filter flags for ball colliding with capsule triggersto add score
			capsule1->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
			capsule2->SetupFiltering(FilterGroup::ACTOR2, FilterGroup::ACTOR0);
			capsule3->SetupFiltering(FilterGroup::ACTOR3, FilterGroup::ACTOR0);
			capsule4->SetupFiltering(FilterGroup::ACTOR4, FilterGroup::ACTOR0);

			pinball->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2 | FilterGroup::ACTOR3 | FilterGroup::ACTOR4);
			
		}

		//Custom update function
		//used to check updates to score/ lives etc.
		virtual void CustomUpdate() 
		{
			//if hit resits pinball
			if (my_callback->outOfBounds == true)
			{
				std::cout << "Life Lost" << endl;
				my_callback->outOfBounds = false;
				ResetPinball();
			}
			//if hit starts the score
			if (my_callback->plungerIsPulled == true)
			{
				Score++;
				std::cout << "PlungerHit" << endl;
			}
			//if hit score increases by 1000 on top of it always incrementing
			if (my_callback->Score == true)
			{
				std::cout << "Score Points" << endl;
				Score = Score += 1000;
				my_callback->Score = false;
			}
			//if hit score increases by 100 on top of it always incrementing
			if (my_callback->Score1 == true)
			{
				std::cout << "Score Points" << endl;
				Score = Score += 100;
				my_callback->Score1 = false;
			}
			//if hit score increases by 250 on top of it always incrementing
			if (my_callback->Score2 == true)
			{
				std::cout << "Score Points" << endl;
				Score = Score += 250;
				my_callback->Score2 = false;
			}
		}

		//Gets the score for the HUD
		virtual int GetScore()
		{
			return Score;
		}

		//resets the pinball machine once the original ball is destroyed
		void ResetPinball()
		{
			px_scene->removeActor(*((PxActor*)pinball->Get()));
			((PxActor*)pinball->Get())->release();
			pinball = new Sphere(PxTransform(PxVec3(18.0f, 18.0f, 23.0f)));
			Add(pinball);
			Lives--;
			if (Lives < 0)
			{
				Lives = 3;
				Score = 0;
			}
			pinball->Color(color_palette[3]);
			pinball->Material(CreateMaterial(0.78f, 0.27f, 0.39f));
			pinball->Name("Pinball");
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am released!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}

		//needed for plunger control, adds force
		void Plunge()
		{
			mainPlunger->PlungerForce(40.0f);
		}

		//needed for plunger control, resets force to zero once released
		void PlungerRelease()
		{
			mainPlunger->PlungerForce(0.0f);
		}

		//flipper left and right control when pressing a button
		void FlipperLeftControl()
		{
			flipperLeftJoint->DriveVelocity(-10);
		}

		void FlipperRightControl()
		{
			flipperRightJoint->DriveVelocity(-10);
		}

		//flipper left and right control when releasing, stops control
		void FlipperLeftRelease()
		{
			flipperLeftJoint->DriveVelocity(10.0f);
		}

		void FlipperRightRelease()
		{
			flipperRightJoint->DriveVelocity(10.0f);
		}
	};
}
