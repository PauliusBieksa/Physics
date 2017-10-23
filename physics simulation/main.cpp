#pragma once
// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <string>
#include <time.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include "glm/ext.hpp"


// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Mesh.h"
#include "Body.h"
#include "RigidBody.h"
#include "Particle.h"


// time
GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

// Struckt for storing spring values
struct spring
{
	float ks;
	float kd;
	float rest;
};



// Constants
const int task = 5;
const glm::vec3 acc_g = glm::vec3(0.0f, -9.8f, 0.0f);
const float bounce_damper = 0.96f;
const spring sp = { 15.0f, 1.0f, 0.3f };	// values for stiffnes, damper, rest distance




// Integrate using semi-implicit Euler integration
void integrate(Particle &p, double dt)
{
	p.setVel(p.getVel() + p.getAcc() * dt);
	p.translate(p.getVel() * dt);
}




// Integrate using semi-implicit Euler integration
void integrate(RigidBody &rb, double dt)
{
	// Angular components
	rb.setAngVel(rb.getAngVel() + rb.getAngAcc() * dt);
	glm::vec3 dRot = rb.getAngVel() * dt;
	if (glm::length2(dRot) > 0.0f)
		rb.rotate(glm::length(dRot), dRot);
	// Non-angular components
	rb.setVel(rb.getVel() + rb.getAcc() * dt);
	rb.translate(rb.getVel() * dt);
}



// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 15.0f));

	// create ground plane
	Mesh plane = Mesh::Mesh();
	// scale it up x5
	plane.scale(glm::vec3(5.0f, 5.0f, 5.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));


	// Make a shader to assign to particles
	Shader shader_green = Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag");
	Shader shader_yellow = Shader("resources/shaders/core.vert", "resources/shaders/core_yellow.frag");

	std::vector<RigidBody> physicsObjects = std::vector<RigidBody>();

	physicsObjects.push_back(RigidBody());
	physicsObjects[0].setMesh(Mesh(Mesh::CUBE));
	physicsObjects[0].getMesh().setShader(shader_green);

	physicsObjects[0].setPos(glm::vec3(0.0f, 5.0f, 0.0f));
	physicsObjects[0].setAngVel(glm::vec3(0.0f, 1.0f, 0.0f));


	// Room corners
	glm::vec3 roomCorner1 = glm::vec3(-10.0f, 0.0f, -10.0f);
	glm::vec3 roomCorner2 = glm::vec3(10.0f, 10.0f, 10.0f);

	// Variables for storing temporary values
	glm::vec3 tmp = glm::vec3(0.0f, 0.0f, 0.0f);


	// Time stuff
	double time = 0.0;
	double dt = 0.01;
	double currentTime = (double)glfwGetTime();
	double timeAccumulator = 0.0;


	// State variables to store positions for interpolation
	std::vector<glm::vec3> prevState;
	std::vector<glm::vec3> currState;
	for (RigidBody &rb : physicsObjects)
		prevState.push_back(rb.getPos());
	for (RigidBody &rb : physicsObjects)
		currState.push_back(rb.getPos());

	double timeFromStart = 0.0f;
	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// Reseting the positions of particles to be calculated on full steps
		for (int i = 0; i < currState.size(); i++)
			physicsObjects[i].setPos(currState[i]);

		// Timekeeping
		double newTime = (double)glfwGetTime();
		double frameTime = newTime - currentTime;
		//frameTime *= 1.25f;	////////////////////////////////////////////////////////////////////////
		if (frameTime > 0.25)
			frameTime = 0.25;
		currentTime = newTime;
		timeAccumulator += frameTime;

		timeFromStart += frameTime;

		// Do fiexed updates while time available
		while (timeAccumulator >= dt)
		{
			// Update prevState
			for (int i = 0; i < physicsObjects.size(); i++)
				prevState[i] = physicsObjects[i].getPos();

			// Apply all forces before integrating
			for (RigidBody &rb : physicsObjects)
			{
				if (rb.isStatic)
					continue;
				rb.setAcc(rb.applyForces(rb.getPos(), rb.getVel()));
			}

			int i = 0;
			// Integrate
			for (RigidBody &rb : physicsObjects)
			{
				integrate(rb, dt);

				int nOfCollisions = 0;
				for (int j = 0; j < rb.getMesh().getVertices().size(); j++)
				{
					for (int k = 0; k < 3; k++)
					{
						if (rb.getMesh().getVertices()[j].getCoord()[k] <= roomCorner1[k])
						{
							nOfCollisions++;
						}
						if (rb.getMesh().getVertices()[j].getCoord()[k] <= roomCorner2[k])
						{
							nOfCollisions++;
						}
					}
				}

				currState[i] = rb.getPos();
				i++;
			}
			timeAccumulator -= dt;
			time += dt;
		}

		const double alpha = timeAccumulator / dt;
		for (int i = 0; i < physicsObjects.size(); i++)
			physicsObjects[i].setPos(alpha * prevState[i] + (1.0 - alpha) * currState[i]);

		/*
		**	INTERACTION
		*/
		// Manage interaction
		app.doMovement(frameTime);


		/*
		**	RENDER
		*/
		// clear buffer
		app.clear();
		// draw groud plane
		app.draw(plane);
		// draw particles
		for (RigidBody rb : physicsObjects)
			app.draw(rb.getMesh());

		app.display();


		// Framerate
		std::cout << 1.0f / frameTime << " fps" << std::endl;
	}


	app.terminate();

	return EXIT_SUCCESS;
}

