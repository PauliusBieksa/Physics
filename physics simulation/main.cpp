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
#include <glm/gtc/quaternion.hpp>
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
#include "BoundingVolume.h"


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
const glm::vec3 acc_g = glm::vec3(0.0f, -9.8f, 0.0f);
//const float bounce_damper = 0.96f;
//const spring sp = { 15.0f, 1.0f, 0.3f };	// values for stiffnes, damper, rest distance



struct physicsObject
{
	RigidBody rb;
	BoundingVolume bv;
};



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
	// Create a skew-simetric matrix for w
	glm::mat3 angVelSkew = glm::matrixCross3(rb.getAngVel());
	// Create 3x3 rotation matrix from rb rotation matrix
	glm::mat3 R = glm::mat3(rb.getRotate());
	// update rotation matrix
	R += dt * angVelSkew * R;
	R = glm::orthonormalize(R);
	rb.setRotate(glm::mat4(R));
	// Non-angular components
	rb.setVel(rb.getVel() + rb.getAcc() * dt);
	rb.translate(rb.getVel() * dt);
}



// Apply impulse / rb - what to apply the impulse to / impulseMagnitude - magnitude of the impulse / r - point of contact - center of mass / normal - collision surface normal
void applyImpulse(RigidBody &rb, float impulseMagnitude, glm::vec3 r, glm::vec3 normal)
{
	rb.setAngVel(rb.getAngVel() + impulseMagnitude * rb.getInvInertia() * glm::cross(r, normal));
	rb.setVel(rb.getVel() + impulseMagnitude / rb.getMass() * normal);
}




void collisionResponse(RigidBody &rb1, RigidBody &rb2, glm::mat2x3 collisionPlane, float intersectionDistance)
{
	intersectionDistance /= 2.0f;
	rb1.translate(-collisionPlane[0] * intersectionDistance);
	rb2.translate(collisionPlane[0] * intersectionDistance);
	glm::vec3 r1 = collisionPlane[1] - rb1.getPos();
	glm::vec3 r2 = collisionPlane[1] - rb2.getPos();
	glm::vec3 vr = rb1.getVel() + glm::cross(rb1.getAngVel(), r1) - (rb2.getVel() + glm::cross(rb2.getAngVel(), r2));
	float jr;
	jr = -(1 + rb1.getCor()) * glm::dot(vr, collisionPlane[0]);		//////////////////// have a look at CoR for both rigid bodies ////////////////
	jr /= 1.0f / rb1.getMass() + 1.0f / rb2.getMass()
		+ glm::dot(collisionPlane[0], glm::cross((rb1.getInvInertia() * glm::cross(r1, collisionPlane[0])), r1))
		+ glm::dot(collisionPlane[0], glm::cross((rb2.getInvInertia() * glm::cross(r2, collisionPlane[0])), r2));

	applyImpulse(rb1, jr, r1, collisionPlane[0]);
	applyImpulse(rb2, -jr, r2, collisionPlane[0]);

	// Friction
	{
		// Coulomb's friction model
		// Calculate tangental velocity
		glm::vec3 vt = vr - glm::dot(vr, glm::vec3(0.0f, 1.0f, 0.0f)) * glm::vec3(0.0f, 1.0f, 0.0f);
		float vtl = glm::length(vt);
		if (vtl == 0.0f)
			return;
		// Calculate tangental impulse
		vt = -0.5f * abs(jr) * vt / vtl;
		// Calculate max friction for rb1
		float jtmax = vtl * rb1.getMass() + glm::length(rb1.getAngVel()) / glm::length(rb1.getInvInertia() * glm::cross(r1, vt / glm::length(vt)));
		if (glm::length2(vt) > jtmax * jtmax)
		{
			vt = vt / glm::length(vt) * jtmax;
			//vtl = jtmax;
		}
		if (glm::length2(vt) > 0.0f)
			applyImpulse(rb1, glm::length(vt), r1, vt / glm::length(vt));
		// Calculate max friction for rb2
		jtmax = vtl * rb2.getMass() + glm::length(rb2.getAngVel()) / glm::length(rb2.getInvInertia() * glm::cross(r2, vt / glm::length(vt)));
		if (glm::length2(vt) > jtmax * jtmax)
		{
			vt = vt / glm::length(vt) * jtmax;
			//vtl = jtmax;
		}
		if (glm::length2(vt) > 0.0f)
			applyImpulse(rb2, glm::length(vt), r2, -vt / glm::length(vt));
	}
}



// Calculates and applies the collision impluse with friction
void applyGroundCollision(RigidBody &rb, glm::vec3 colPoint)
{
	glm::vec3 r = colPoint - rb.getPos();
	// Calculate relative velocity
	glm::vec3 vr = rb.getVel() + glm::cross(rb.getAngVel(), r);

	// Reduce collision response and reduce spinning when the whelocity of an object is below a threshold
	float cor = rb.getCor();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//if (glm::length2(vr) < 1.0f && glm::length2(rb.getVel()) < 0.64f)
	//{
	//	cor *= glm::length2(vr);
	//	if (glm::length2(vr) < 0.01f && glm::length2(rb.getVel()) < 0.01f)
	//		cor = 0.0f;
	//	// Recalculate relative velocity
	//	//	tmp = rb.getVel() + glm::cross(rb.getAngVel(), r);

	//	// If the velocity og the object is below a threshold, make it static
	//	if (glm::length2(rb.getVel()) < 0.025f && glm::length2(rb.getAngVel()) < 0.025f)
	//		rb.isStatic = true;
	//}
	// Calculate magnitude of the impulse
	float jr = -1.0f * (1.0f + cor) * glm::dot(vr, glm::vec3(0.0f, 1.0f, 0.0f));
	jr /= 1.0f / rb.getMass() + glm::dot(glm::vec3(0.0f, 1.0f, 0.0f), glm::cross(rb.getInvInertia() * glm::cross(r, glm::vec3(0.0f, 1.0f, 0.0f)), r));
	// Apply the impulse
	applyImpulse(rb, jr, r, glm::vec3(0.0f, 1.0f, 0.0f));

	// Coulomb's friction model
	// Calculate tangental velocity
	glm::vec3 vt = vr - glm::dot(vr, glm::vec3(0.0f, 1.0f, 0.0f)) * glm::vec3(0.0f, 1.0f, 0.0f);
	float vtl = glm::length(vt);
	if (vtl == 0.0f)
		return;
	// Calculate tangental impulse
	vt = -0.6f * abs(jr) * vt / vtl;
	// Calculate max friction
	float jtmax = vtl * rb.getMass() + glm::length(rb.getAngVel()) / glm::length(rb.getInvInertia() * glm::cross(r, vt / glm::length(vt)));
	if (glm::length2(vt) > jtmax * jtmax)
	{
		vt = vt / glm::length(vt) * jtmax;
		//vtl = jtmax;
	}
	if (glm::length2(vt) > 0.0f)
		applyImpulse(rb, glm::length(vt), r, vt / glm::length(vt));
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
	plane.setShader(Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag"));


	// Make a shader to assign to particles
	Shader shader_green = Shader("resources/shaders/physics.vert", "resources/shaders/physics_green.frag");
	Shader shader_yellow = Shader("resources/shaders/core.vert", "resources/shaders/core_yellow.frag");
	Shader shader_red = Shader("resources/shaders/core.vert", "resources/shaders/core_red.frag");


	// Debug particle ///////////////////////////
	Particle debugParticle = Particle();
	debugParticle.getMesh().setShader(shader_red);
	Particle debugParticle1 = Particle();
	debugParticle1.getMesh().setShader(shader_yellow);

	//std::vector<RigidBody> physicsObjects = std::vector<RigidBody>();
	//// Rigid bodies
	//physicsObjects.push_back(RigidBody());
	//physicsObjects[0].setMesh(Mesh(Mesh::CUBE));
	//physicsObjects[0].getMesh().setShader(shader_green);
	//physicsObjects[0].setMass(2.0f);
	//physicsObjects[0].scale(glm::vec3(1.0f, 6.0f, 2.0f));
	//physicsObjects[0].setPos(glm::vec3(-4.0f, 5.0f, 0.0f));
	//physicsObjects[0].setCor(0.5f);
	//physicsObjects[0].setVel(glm::vec3(0.8f, 0.0f, 0.0f));
	//physicsObjects[0].setAngVel(glm::vec3(0.0f, 0.0f, 0.3f));
	//physicsObjects[0].addForce(new Gravity());
	//BoundingVolume bv1 = BoundingVolume(physicsObjects[0].getPos(), physicsObjects[0].getRotate()
	//	, glm::vec3(physicsObjects[0].getScale()[0][0], physicsObjects[0].getScale()[1][1], physicsObjects[0].getScale()[2][2]) / 2.0f);

	//physicsObjects.push_back(RigidBody());
	//physicsObjects[1].setMesh(Mesh(Mesh::CUBE));
	//physicsObjects[1].getMesh().setShader(shader_green);
	//physicsObjects[1].setMass(2.0f);
	//physicsObjects[1].scale(glm::vec3(1.0f, 6.0f, 2.0f));
	//physicsObjects[1].setPos(glm::vec3(0.0f, 5.0f, 0.0f));
	//physicsObjects[1].setCor(0.5f);
	//physicsObjects[1].addForce(new Gravity());
	//physicsObjects[1].setVel(glm::vec3(-0.8f, 0.0f, 0.0f));
	////physicsObjects[1].rotate(glm::quarter_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
	////physicsObjects[1].rotate(glm::quarter_pi<float>() / 2.0f, glm::vec3(0.0f, 0.0f, 1.0f));
	//BoundingVolume bv2 = BoundingVolume(physicsObjects[1].getPos(), physicsObjects[1].getRotate()
	//	, glm::vec3(physicsObjects[1].getScale()[0][0], physicsObjects[1].getScale()[1][1], physicsObjects[1].getScale()[2][2]) / 2.0f);

	std::vector<physicsObject> physicsObjects;
	for (int i = 0; i < 5; i++)
	{
		physicsObjects.push_back(physicsObject());
		physicsObjects[i].rb = RigidBody();
		physicsObjects[i].rb.setMesh(Mesh(Mesh::CUBE));
		physicsObjects[i].rb.getMesh().setShader(shader_green);
		physicsObjects[i].rb.setMass(2.0f);
		physicsObjects[i].rb.setCor(0.5f);
		physicsObjects[i].rb.scale(glm::vec3(0.25f, 1.5f, 0.5f));
		physicsObjects[i].rb.setPos(glm::vec3(-5.0f + (i / 1.0f), 1.5f, 0.0f));
		physicsObjects[i].rb.addForce(new Gravity());
		physicsObjects[i].bv = BoundingVolume(physicsObjects[0].rb.getPos(), physicsObjects[0].rb.getRotate()
			, glm::vec3(physicsObjects[0].rb.getScale()[0][0], physicsObjects[0].rb.getScale()[1][1], physicsObjects[0].rb.getScale()[2][2]) / 2.0f);
	}

	// Room corners
	glm::vec3 roomCorner1 = glm::vec3(-10.0f, 0.0f, -10.0f);
	glm::vec3 roomCorner2 = glm::vec3(10.0f, 10.0f, 10.0f);



	////////////////////////////////////////////////////////////////////////////////
	// Variables for storing temporary values
	glm::vec3 tmp = glm::vec3(0.0f, 0.0f, 0.0f);

	// Time stuff
	double time = 0.0;
	double dt = 0.01;
	double currentTime = (double)glfwGetTime();
	double timeAccumulator = 0.0;

	// State variables to store positions for interpolation
	//std::vector<glm::mat4> prevState;
	//std::vector<glm::mat4> currState;
	////std::vector<glm::vec3> prevState;
	////std::vector<glm::vec3> currState;
	//for (RigidBody &rb : physicsObjects)
	//	prevState.push_back(rb.getTranslate() * rb.getRotate());
	//for (RigidBody &rb : physicsObjects)
	//	currState.push_back(rb.getTranslate() * rb.getRotate());
	/*
	for (RigidBody &rb : physicsObjects)
		prevState.push_back(rb.getPos());
	for (RigidBody &rb : physicsObjects)
		currState.push_back(rb.getPos());*/

	double timeFromStart = 0.0f;
	///////////////////////////////////////////////////////////////////////////////


	bool paused = false;
	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// Reseting the positions of particles to be calculated on full steps
		//for (int i = 0; i < currState.size(); i++)
		//{
		//	physicsObjects[i].setPos(currState[i][3]);
		//	// Set rotate without the translate values
		//	physicsObjects[i].setRotate(glm::mat4(glm::mat3(currState[i])));
		//}

		// Timekeeping
		double newTime = (double)glfwGetTime();
		double frameTime = newTime - currentTime;
		timeFromStart += frameTime;
		//frameTime *= 0.5f;	//////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (frameTime > 0.25)
			frameTime = 0.25;
		currentTime = newTime;
		if (!app.pauseSimulation)
		{
			timeAccumulator += frameTime;
		}

		// Do fixed updates while time available
		//while (timeAccumulator >= dt && !paused)
		while (timeAccumulator >= dt)
		{
			// Update prevState
			/*for (int i = 0; i < physicsObjects.size(); i++)
			{
				prevState[i] = physicsObjects[i].getTranslate() * physicsObjects[i].getRotate();
			}*/

			// Apply all forces before integrating
			for (physicsObject &p : physicsObjects)
			{
				if (p.rb.isStatic)
					continue;
				p.rb.setAcc(p.rb.applyForces(p.rb.getPos(), p.rb.getVel()));
			}

			int i = 0;
			// Integrate
			for (physicsObject &p : physicsObjects)
			{
				// Ignore objects marked as static
				if (p.rb.isStatic)
					continue;

				integrate(p.rb, dt);
			}

			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			for (physicsObject &p : physicsObjects)
			{
				if (p.bv.getColliderType() == BoundingVolume::OBB)
					p.bv.updateOBB(p.rb.getPos(), p.rb.getRotate());
			}
			if (physicsObjects.size() > 1)
				for (int i = physicsObjects.size() - 1; i > 1; i--)
					for (int j = i - 1; j > 0; j--)
					{
						std::pair<glm::mat2x3, float> result = physicsObjects[i].bv.collisionCheck(physicsObjects[j].bv);
						glm::mat2x3 pl = result.first;
						float d = result.second;
						if (pl[0][0] == pl[0][0])
						{
							debugParticle.setPos(pl[1]);
							debugParticle1.setPos(pl[1] + pl[0]);
							collisionResponse(physicsObjects[i].rb, physicsObjects[i].rb, pl, d);
						}
					}


			//bv1.updateOBB(physicsObjects[0].getPos(), physicsObjects[0].getRotate());
			//bv2.updateOBB(physicsObjects[1].getPos(), physicsObjects[1].getRotate());
			//std::pair<glm::mat2x3, float> result = bv1.collisionCheck(bv2);
			//glm::mat2x3 pl = result.first;
			//if (pl[0][0] == pl[0][0]) //////////////// 2 collision checks ///////////////////
			//{
			//	debugParticle.setPos(pl[1]);
			//	debugParticle1.setPos(pl[1] + pl[0]);
			//	collisionResponse(physicsObjects[0], physicsObjects[1], pl, result.second);
			//}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////



			// Collisions with the ground
			for (physicsObject &p : physicsObjects)
			{
				// Collision detection
				int nOfCollisions = 0;
				std::vector<glm::vec3> collisionPoints = std::vector<glm::vec3>();
				std::vector<glm::vec3> worldVertices = std::vector<glm::vec3>();
				for (int j = 0; j < p.rb.getMesh().getVertices().size(); j++)
				{
					worldVertices.push_back(p.rb.getMesh().getModel() * glm::vec4(p.rb.getMesh().getVertices()[j].getCoord(), 1.0f));
					worldVertices[j] /= (p.rb.getMesh().getModel() * glm::vec4(p.rb.getMesh().getVertices()[j].getCoord(), 1.0f)).w;
				}
				// Move the rigid body above gorund
				tmp = glm::vec3(0.0f);
				for (glm::vec3 wv : worldVertices)
					if (wv[1] < tmp[1])
						tmp = wv;
				if (tmp[1] < 0.0f)
					p.rb.translate(glm::vec3(0.0f, -tmp[1], 0.0f));
				// Count number of collisions
				for (glm::vec3 wv : worldVertices)
					if (wv[1] <= 0.0f)
					{
						collisionPoints.push_back(wv);
						nOfCollisions++;
					}
				tmp = glm::vec3(0.0f);
				for (glm::vec3 colPoint : collisionPoints)
					tmp += colPoint;
				tmp /= nOfCollisions;
				glm::vec3 colMidpoint = tmp;

				// Apply impulses to collision points
				if (nOfCollisions > 0)
				{
					applyGroundCollision(p.rb, colMidpoint);
				}

				/*currState[i] = rb.getTranslate() * rb.getRotate();*/
				i++;
			}
			timeAccumulator -= dt;
			time += dt;
		}

		//if (!paused)
		//{
		//	const double alpha = timeAccumulator / dt;
		//	for (int i = 0; i < physicsObjects.size(); i++)
		//	{
		//		physicsObjects[i].setPos(alpha * prevState[i][3] + (1.0 - alpha) * currState[i][3]);
		//		
		//		//physicsObjects[i].setRotate(M);
		//	}
		//}

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
		for (physicsObject p : physicsObjects)
			app.draw(p.rb.getMesh());

		app.draw(debugParticle.getMesh());
		app.draw(debugParticle1.getMesh());
		app.display();


		// Framerate
		std::cout << 1.0f / frameTime << " fps" << std::endl;
	}


	app.terminate();

	return EXIT_SUCCESS;
}

