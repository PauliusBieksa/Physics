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



glm::vec3 * wind;




// Integrate using semi-implicit Euler integration
void integrate(Particle &p, float dt)
{
	p.setVel(p.getVel() + p.getAcc() * dt);
	p.translate(p.getVel() * dt);
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

	// Vector that store particles
	std::vector<Particle> particles;

	// Make a shader to assign to particles
	Shader shader_particle = Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag");
	Shader shader_yellow = Shader("resources/shaders/core.vert", "resources/shaders/core_yellow.frag");

	// Task 1
	if (task == 1)
	{
		// Adding particles
		for (int i = 0; i < 5; i++)
		{
			particles.push_back(Particle());
			particles[i].translate(glm::vec3(0.0f + (float)i / 2.0f, 4.5f, 0.0f));
			if (i == 0)
				particles[i].getMesh().setShader(shader_yellow);
			else
			{
				particles[i].getMesh().setShader(shader_particle);
				particles[i].setMass(0.1f);
				particles[i].addForce(new Gravity());
				particles[i].addForce(new Drag());
			}
		}
		// Adding springs
		particles[1].addForce(new Hooke(&particles[0], sp.ks, sp.kd, sp.rest));
		for (int i = 1; i < particles.size() - 1; i++)
		{
			particles[i].addForce(new Hooke(&particles[i + 1], sp.ks, sp.kd, sp.rest));
			particles[i + 1].addForce(new Hooke(&particles[i], sp.ks, sp.kd, sp.rest));
		}
	}

	// Tasks 2 and 3
	if (task == 2 || task == 3)
	{
		// Adding particles
		for (int i = 0; i < 10; i++)
		{
			particles.push_back(Particle());
			if (task == 2)
				particles[i].translate(glm::vec3(-2.5f + (float)i / 2.0f, 4.5f, 0.0f));
			else if (task == 3)
				particles[i].translate(glm::vec3(-2.5f + (float)i / 2.0f, 1.0f, 0.0f));
			if (i == 0 || i == 9)
				particles[i].getMesh().setShader(shader_yellow);
			else
			{
				particles[i].getMesh().setShader(shader_particle);
				particles[i].setMass(0.1f);
				particles[i].addForce(new Gravity());
				particles[i].addForce(new Drag());
			}
		}
		// Adding springs
		particles[1].addForce(new Hooke(&particles[0], sp.ks, sp.kd, sp.rest));
		particles[8].addForce(new Hooke(&particles[9], sp.ks, sp.kd, sp.rest));
		for (int i = 1; i < particles.size() - 2; i++)
		{
			particles[i].addForce(new Hooke(&particles[i + 1], sp.ks, sp.kd, sp.rest));
			particles[i + 1].addForce(new Hooke(&particles[i], sp.ks, sp.kd, sp.rest));
		}
	}

	// task 4
	if (task == 4)
	{
		// Adding particles
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				particles.push_back(Particle());
				particles[i * 10 + j].translate(glm::vec3(-2.5f + (float)i / 2.0f, 2.5f, 2.5f - (float)j / 2.0f));
				if ((i == 0 && j == 0) || (i == 9 && j == 0) || (i == 0 && j == 9) || (i == 9 && j == 9))
				{
					particles[i * 10 + j].getMesh().setShader(shader_yellow);
					continue;
				}
				particles[i * 10 + j].getMesh().setShader(shader_particle);
				particles[i * 10 + j].setMass(0.1f);
				particles[i * 10 + j].addForce(new Gravity());
				particles[i * 10 + j].addForce(new Drag());
			}
		}
		// Adding springs
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				if ((i == 0 && j == 0) || (i == 9 && j == 0) || (i == 0 && j == 9) || (i == 9 && j == 9))
					continue;

				if (i > 0)
					particles[i * 10 + j].addForce(new Hooke(&particles[(i - 1) * 10 + j], sp.ks, sp.kd, sp.rest));
				if (i < 9)
					particles[i * 10 + j].addForce(new Hooke(&particles[(i + 1) * 10 + j], sp.ks, sp.kd, sp.rest));
				if (j > 0)
					particles[i * 10 + j].addForce(new Hooke(&particles[i * 10 + j - 1], sp.ks, sp.kd, sp.rest));
				if (j < 9)
					particles[i * 10 + j].addForce(new Hooke(&particles[i * 10 + j + 1], sp.ks, sp.kd, sp.rest));
			}
		}
	}

	// Task 5
	wind = new glm::vec3(0.0f, 0.0f, 5.5f);
	if (task == 5)
	{
		// Adding particles
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				particles.push_back(Particle());
				particles[i * 10 + j].translate(glm::vec3(-2.5f + (float)i / 2.0f, 7.0f - (float)j / 2.0f, 0.0f));
				if ((i == 0 && j == 0) || (i == 9 && j == 0))
				{
					particles[i * 10 + j].getMesh().setShader(shader_yellow);
					continue;
				}
				particles[i * 10 + j].getMesh().setShader(shader_particle);
				particles[i * 10 + j].setMass(0.1f);
				particles[i * 10 + j].addForce(new Gravity());
			}
		}
		// Adding springs and surface drag
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				// Adding surface drag
				if (i < 9 && j < 9)
				{
					Force *f1 = new SurfaceDrag(&particles[i * 10 + j], &particles[(i + 1) * 10 + j], &particles[(i + 1) * 10 + j + 1], wind);
					Force *f2 = new SurfaceDrag(&particles[i * 10 + j], &particles[(i + 1) * 10 + j + 1], &particles[i * 10 + j + 1], wind);
					particles[i * 10 + j].addForce(f1);
					particles[(i + 1) * 10 + j].addForce(f1);
					particles[(i + 1) * 10 + j + 1].addForce(f1);
					particles[i * 10 + j].addForce(f2);
					particles[(i + 1) * 10 + j + 1].addForce(f2);
					particles[i * 10 + j + 1].addForce(f2);
				}

				if ((i == 0 && j == 0) || (i == 9 && j == 0))
				{
					particles[i * 10 + j].isStatic = true;
					continue;
				}

				// Adding springs
				if (i > 0)
					particles[i * 10 + j].addForce(new Hooke(&particles[(i - 1) * 10 + j], sp.ks, sp.kd, sp.rest));
				if (j > 0)
					particles[i * 10 + j].addForce(new Hooke(&particles[i * 10 + j - 1], sp.ks, sp.kd, sp.rest));
				if (i < 9)
					particles[i * 10 + j].addForce(new Hooke(&particles[(i + 1) * 10 + j], sp.ks, sp.kd, sp.rest));
				if (j < 9)
					particles[i * 10 + j].addForce(new Hooke(&particles[i * 10 + j + 1], sp.ks, sp.kd, sp.rest));
			}
		}
	}


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
	for (Particle &p : particles)
		prevState.push_back(p.getPos());
	for (Particle &p : particles)
		currState.push_back(p.getPos());

	float timeFromStart = 0.0f;
	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// Reseting the positions of particles to be calculated on full steps
		for (int i = 0; i < currState.size(); i++)
			particles[i].setPos(currState[i]);

		// Timekeeping
		double newTime = (double)glfwGetTime();
		double frameTime = newTime - currentTime;
		//frameTime *= 0.25f;	////////////////////////////////////////////////////////////////////////
		if (frameTime > 0.25)
			frameTime = 0.25;
		currentTime = newTime;
		timeAccumulator += frameTime;

		timeFromStart += frameTime;

		// Do fiexed updates while time available
		while (timeAccumulator >= dt)
		{
			*wind = glm::vec3(glm::sin(timeFromStart / 5.0f) * 6.0f, 0.0f, glm::cos(timeFromStart / 5.0f) * 6.0f);
			for (int i = 0; i < prevState.size(); i++)
				prevState[i] = particles[i].getPos();

			// Apply all forces before integrating
			for (Particle &p : particles)
			{
				if (p.isStatic)
					continue;
				p.setAcc(p.applyForces(p.getPos(), p.getVel()));
			}

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
		app.doMovement(frameTime);


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

