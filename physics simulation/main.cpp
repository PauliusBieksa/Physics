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
// Constants describing blowdryers cone of effect
const float blowConeRTop = 2.0f;
const float blowConeRbot = 1.0f;
const float blowConeHeight = 3.5f;
const float blowMaxSpeed = 100.0f;

int task = 4;
bool debugParticles = false;




// Calculates drag
glm::vec3 drag(float area, float dragCoefficient, glm::vec3 velocity)
{
	float air_density = 1.2f;
	return (velocity * -0.5f * area * dragCoefficient * air_density * velocity.length());
}



// Integrate using forward Euler integration
void integrateForward(Particle &p, glm::vec3 combinedForce, float dt)
{
	glm::vec3 v;
	v = p.getVel() + p.getAcc() * dt;
	p.setAcc(acc_g + combinedForce / p.getMass());
	if (v.length() < 0.05f)
		v = glm::vec3(0.0f, 0.0f, 0.0f);
	p.setPos(p.getPos() + p.getVel() * dt);
	p.setVel(v);
}



// Calculates the blowdryer wind speed
glm::vec3 windSpeed(glm::vec3 pos)
{
	float windSpeedHere = blowMaxSpeed;
	if (pos[1] < blowConeHeight)
	{
		// Pythagoras to find out where the wind is pointing to
		float r_at_h = (blowConeRTop - blowConeRbot) * pos[1] / blowConeHeight + blowConeRbot;
		float posProjOnXZLength = sqrtf(pos[0] * pos[0] + pos[2] * pos[2]);
		if (r_at_h <= posProjOnXZLength)
			return glm::vec3();

		glm::vec3 pointingTo;
		if (pos[0] == 0.0f && pos[2] == 0)
			pointingTo = glm::vec3(0.0f, blowConeHeight, 0.0f);
		else
			pointingTo = normalize(glm::vec3(pos[0], 0.0f, pos[2])) * blowConeRTop * (posProjOnXZLength / r_at_h);
		pointingTo[1] = blowConeHeight;

		glm::vec3 direction = normalize(pointingTo - pos);

		windSpeedHere *= cosf(glm::pi<float>() / 2.0f * (blowConeHeight / (pos[1] == 0.0f) ? 0.00000001f : pos[1]));
		windSpeedHere *= cosf(glm::pi<float>() / 2.0f * (posProjOnXZLength / r_at_h));
		direction *= windSpeedHere;
		return direction;
	}
	return glm::vec3();
}



// Integrate using semi-implicit Euler integration
void integrate(Particle &p, glm::vec3 combinedForce, float dt)
{
	p.setAcc(acc_g + combinedForce / p.getMass());
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


	// Particles for demonstrating different integration methods
	std::vector<Particle> task3;
	task3.push_back(Particle());
	task3[0].translate(glm::vec3(-0.5f, 2.0f, 0.0f));
	task3[0].getMesh().setShader(shader_particle);

	task3.push_back(Particle());
	task3[1].translate(glm::vec3(0.0f, 2.0f, 0.0f));
	task3[1].getMesh().setShader(shader_particle);

	task3.push_back(Particle());
	task3[2].translate(glm::vec3(0.5f, 2.0f, 0.0f));
	task3[2].getMesh().setShader(shader_particle);


	// Room corners
	glm::vec3 roomCorner1 = glm::vec3(-2.5f, 0.0f, -2.5f);
	glm::vec3 roomCorner2 = glm::vec3(2.5f, 5.0f, 2.5f);

	// Variables for storing temporary values
	glm::vec3 f_drag = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 tmp = glm::vec3(0.0f, 0.0f, 0.0f);

	Particle shadow = Particle();
	shadow.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_yellow.frag"));
	Particle dragThere = Particle();
	dragThere.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_red.frag"));

	// Time stuff
	double time = 0.0;
	//double dt = 1.0 / 60.0;
	double dt = 0.01;
	double currentTime = (double)glfwGetTime();
	double timeAccumulator = 0.0;


	// State variables to store positions for interpolation
	std::vector<glm::vec3> prevState;
	std::vector<glm::vec3> currState;
	if (task == 4)
	{
		for (Particle &p : particles)
			prevState.push_back(p.getPos());
		for (Particle &p : particles)
			currState.push_back(p.getPos());
	}
	else
	{
		for (Particle &p : task3)
			prevState.push_back(p.getPos());
		for (Particle &p : task3)
			currState.push_back(p.getPos());
	}


	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// Reseting the positions of particles to be calculated on full steps
		if (task == 4)
			for (int i = 0; i < currState.size(); i++)
				particles[i].setPos(currState[i]);
		else
			for (int i = 0; i < currState.size(); i++)
				task3[i].setPos(currState[i]);

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
			if (task == 4)
			{
				// Task 4
				for (int i = 0; i < prevState.size(); i++)
					prevState[i] = particles[i].getPos();

				int i = 0;
				for (Particle &p : particles)
				{
					f_drag = drag(0.01f, 1.2f, p.getVel() - windSpeed(p.getPos()));
					integrate(p, f_drag, dt);

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

					// Uses a red particle to show the drag vector and a yellow particle to show an orthographic "shadow" on the floor
					dragThere.setPos(p.getPos() + f_drag * 5.0f);
					shadow.setPos(glm::vec3(p.getPos()[0], 0.0f, p.getPos()[2]));
				}
			}
			else
			{
				// Task 3
				for (int i = 0; i < prevState.size(); i++)
					prevState[i] = task3[i].getPos();

				int i = 0;
				for (Particle &p : task3)
				{
					// task 3 is done with no drag
					if (i == 1)
						integrate(p, glm::vec3(), dt);
					else if (i == 2)
						integrateForward(p, glm::vec3(), dt);

					// Collision detection
					for (int i = 0; i < 3; i++)
						if (p.getTranslate()[3][i] <= roomCorner1[i])
						{
							tmp = p.getPos();
							tmp[i] = roomCorner1[i];
							p.setPos(tmp);
							tmp = p.getVel();
							tmp[i] = abs(p.getVel()[i]);
							p.setVel(tmp);
						}
						else if (p.getTranslate()[3][i] >= roomCorner2[i])
						{
							tmp = p.getPos();
							tmp[i] = roomCorner2[i];
							p.setPos(tmp);
							tmp = p.getVel();
							tmp[i] = -abs(p.getVel()[i]);
							p.setVel(tmp);
						}

					currState[i] = p.getPos();
					i++;
				}
			}

			timeAccumulator -= dt;
			time += dt;
		}

		const double alpha = timeAccumulator / dt;
		if (task == 4)
			for (int i = 0; i < particles.size(); i++)
				particles[i].setPos(alpha * prevState[i] + (1.0 - alpha) * currState[i]);
		else
			for (int i = 0; i < task3.size(); i++)
				task3[i].setPos(alpha * prevState[i] + (1.0 - alpha) * currState[i]);

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
		if (task == 3)
			for (Particle p : task3)
				app.draw(p.getMesh());
		else
			for (Particle p : particles)
				app.draw(p.getMesh());
		if (debugParticles)
		{
			app.draw(shadow.getMesh());
			app.draw(dragThere.getMesh());
		}

		app.display();


		// Framerate
		std::cout << 1.0f / frameTime << " fps" << std::endl;
	}


	app.terminate();

	return EXIT_SUCCESS;
}

