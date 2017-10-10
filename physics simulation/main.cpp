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
#include "Particle.h"


// time
GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

// Cosntants
const glm::vec3 acc_g = glm::vec3(0.0f, -9.8f, 0.0f);
const float bounce_damper = 0.96f;






// Integrate using semi-implicit Euler integration
void integrate(Particle &p, float dt)
{
	p.setAcc(p.applyForces(p.getPos(), p.getVel()));
	p.setVel(p.getVel() + p.getAcc() * dt);
	if (p.getVel().length() < 0.05f)
		p.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	p.translate(p.getVel() * dt);
}



// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 20.0f));

	// create ground plane
	Mesh plane = Mesh::Mesh();
	// scale it up x5
	plane.scale(glm::vec3(5.0f, 5.0f, 5.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));

	// Vector that store particles
	std::vector<Particle> particles;

	// Make a shader to assign to particles
	Shader shader_particle = Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag");

	// Set particle parameters
	particles.push_back(Particle());
	particles[0].translate(glm::vec3(0.0f, 4.5f, 0.0f));
	particles[0].getMesh().setShader(shader_particle);
	particles[0].setVel(glm::vec3(4.0f, 1.0f, 1.0f));
	particles[0].setMass(0.1f);
	Gravity fg = Gravity();
	particles[0].addForce(&fg);
	Drag fd = Drag();
	particles[0].addForce(&fd);

	// Room corners
	glm::vec3 roomCorner1 = glm::vec3(-2.5f, 0.0f, -2.5f);
	glm::vec3 roomCorner2 = glm::vec3(2.5f, 5.0f, 2.5f);

	// Variables for storing temporary values
	glm::vec3 tmp = glm::vec3(0.0f, 0.0f, 0.0f);


	// Time stuff
	double time = 0.0;
	//double dt = 1.0 / 60.0;
	double dt = 0.01;
	double currentTime = (double)glfwGetTime();
	double timeAccumulator = 0.0;


	// State variables to store positions for interpolation
	std::vector<glm::vec3> prevState;
	std::vector<glm::vec3> currState;
	for (Particle &p : particles)
		prevState.push_back(p.getPos());
	for (Particle &p : particles)
		currState.push_back(p.getPos());

	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// Reseting the positions of particles to be calculated on full steps
		for (int i = 0; i < currState.size(); i++)
			particles[i].setPos(currState[i]);

		// Timekeeping
		double newTime = (double)glfwGetTime();
		double frameTime = newTime - currentTime;
		if (frameTime > 0.25)
			frameTime = 0.25;
		currentTime = newTime;
		timeAccumulator += frameTime;

		// Do fiexed updates while time available
		while (timeAccumulator >= dt)
		{
			for (int i = 0; i < prevState.size(); i++)
				prevState[i] = particles[i].getPos();

			int i = 0;
			for (Particle &p : particles)
			{
				integrate(p, dt);

				// Collision detection
				for (int i = 0; i < 3; i++)
					if (p.getTranslate()[3][i] <= roomCorner1[i])
					{
						tmp = p.getPos();
						tmp[i] = roomCorner1[i];
						p.setPos(tmp);
						tmp = p.getVel();
						tmp[i] = abs(p.getVel()[i]) * bounce_damper;
						p.setVel(tmp);
					}
					else if (p.getTranslate()[3][i] >= roomCorner2[i])
					{
						tmp = p.getPos();
						tmp[i] = roomCorner2[i];
						p.setPos(tmp);
						tmp = p.getVel();
						tmp[i] = -abs(p.getVel()[i]) * bounce_damper;
						p.setVel(tmp);
					}
				currState[i] = p.getPos();
				i++;
			}
			timeAccumulator -= dt;
			time += dt;
		}

		const double alpha = timeAccumulator / dt;
		for (int i = 0; i < particles.size(); i++)
			particles[i].setPos(alpha * prevState[i] + (1.0 - alpha) * currState[i]);

		/*
		**	INTERACTION
		*/
		// Manage interaction
		app.doMovement(timeAccumulator * 2.0f);


		/*
		**	RENDER
		*/
		// clear buffer
		app.clear();
		// draw groud plane
		app.draw(plane);
		// draw particles
		for (Particle p : particles)
			app.draw(p.getMesh());

		app.display();


		// Framerate
		std::cout << 1.0f / frameTime << " fps" << std::endl;
	}


	app.terminate();

	return EXIT_SUCCESS;
}

