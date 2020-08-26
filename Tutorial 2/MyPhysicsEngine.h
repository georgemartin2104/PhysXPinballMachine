#pragma once

/*
	BRIEF DESCRIPTION OF CHANGES:

	The code provided implements a pinball machine case made of a compound object,
	it also includes a pinball, two flippers, two bumpers and a spinner.

	The machine, pinball, flippers, bumper and spinner all have their own materials,
	and in the case of the machine it has a different material for the floor to the
	walls.

	There is a scoring mechanism based on how long the game is played that is reset
	when the plunger is used. It also tracks the player's high score.

	There is a small use of convex meshes which has been used to implement triangular prisms
	under the legs of the machine to make it sit flat.

	The flippers use a revolute joint and the plunger uses a distance joint.
*/


#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	///Custom scene class
	class MyScene : public Scene
	{
		// Plane for the floor (for the machine to rest on)
		Plane* plane;

		// Pinball
		Pinball* pinball;

		// A list of objects
		PinballCase* pinballCase;
		Plunger* plunger;
		Flipper* leftFlipper;
		Flipper* rightFlipper;
		Bumper* bumper1;
		Bumper* bumper2;
		Spinner* spinner;

		// A list of materials
		PxMaterial* ballMaterial;


	public:

		// Storage values
		int score = 0;
		int highScore = 0;
		int timer = 0;


		///A custom scene class
		void SetVisualisation()
		{
			// Set debugging visualisations to appear when F7 is pressed
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eBODY_LIN_VELOCITY, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();		

			// Create Material for the ball
			ballMaterial = CreateMaterial(0.f, 0.f, 0.5f);

			// Create the plane
			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			Add(plane);
			
			// Create the pinball case
			pinballCase = new PinballCase(PxTransform(PxVec3(0.f, 3.6f, 0.f), PxQuat(-1.39626f, PxVec3(1.f, .0f, .0f))));
			pinballCase->Color(PxVec3(55.f, 0.f, 0.f));
			Add(pinballCase);

			// Create the pinball
			pinball = new Pinball(PxTransform(PxVec3(3.8f, 5.3f, 4.f)));
			pinball->Color(PxVec3(0.f, 0.f, 0.f));
			pinball->Material(ballMaterial);
			Add(pinball);

			// Create the plunger
			plunger = new Plunger(PxTransform(PxVec3(4.2f, 2.2f, 12.f), PxQuat(-1.39626f, PxVec3(1.f, .0f, .0f))));
			plunger->AddToScene(this);


			// Create the flippers
			leftFlipper = new Flipper(true, PxTransform(PxVec3(-2.28f, 3.7f, 5.7f)));
			((PxRigidActor*)leftFlipper->Get())->setName("leftFlipper");
			Add(leftFlipper);

			rightFlipper = new Flipper(false, PxTransform(PxVec3(.78f, 3.7f, 5.7f)));
			Add(rightFlipper);

			// Create the bumpers
			bumper1 = new Bumper(PxTransform(PxVec3(1.f, 4.2f, 0.f), PxQuat(0.174533f, PxVec3(1.f, .0f, .0f))));
			bumper1->Color(PxVec3(0.f, 55.f, 0.f));
			Add(bumper1);

			bumper2 = new Bumper(PxTransform(PxVec3(-2.5f, 4.2f, 0.f), PxQuat(0.174533f, PxVec3(1.f, .0f, .0f))));
			bumper2->Color(PxVec3(0.f, 55.f, 0.f));
			Add(bumper2);

			// Create the spinner
			spinner = new Spinner(PxTransform(PxVec3(-.6f, 5.2f, -5.f), PxQuat(0.174533f, PxVec3(1.f, .0f, .0f))));
			spinner->Color(PxVec3(0.f, 0.f, 55.f));
			spinner->SetKinematic(true);
			Add(spinner);
		}

		//Custom update function
		virtual void CustomUpdate() 
		{
			// Add to the timer once per frame
			timer++;

			// Every 100 frames add score
			if (timer >= 100)
			{
				score += 100;
				timer = 0;
			}
			// Tracks highscore
			if (highScore <= score)
			{
				highScore = score;
			}

			// Makes the spinner spin
			PxTransform spinnerPose = ((PxRigidBody*)spinner->Get())->getGlobalPose();
			spinnerPose = spinnerPose.getNormalized();
			spinnerPose.q = spinnerPose.q * PxQuat(.1f, PxVec3(.0f, 1.0f, .0f));
			((PxRigidBody*)spinner->Get())->setGlobalPose(spinnerPose, true);
		}

		// Called when plunger is used
		void UsePlunger()
		{
			plunger->use();
			score = 0;	// When the plunger is used, reset the score
		}

		// Called when flipper is used
		void UseFlipper(bool isLeftFlipper, PxReal speed)
		{
			// Add velocity to the joints of the flippers
			if (isLeftFlipper)
			{
				leftFlipper->GetJoint()->DriveVelocity(speed);
			}
			if (!isLeftFlipper)
			{
				rightFlipper->GetJoint()->DriveVelocity(speed);
			}
		}
	};
}
