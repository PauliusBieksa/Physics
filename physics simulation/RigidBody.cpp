// Math constants
# define _USE_MATH_DEFINES
# include <cmath>
# include "RigidBody.h"


RigidBody::RigidBody()
{

	// set dynamic values
	setAcc(glm::vec3(0.0f, 0.0f, 0.0f));
	setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
	setAngAccl(glm::vec3(0.0f, 0.0f, 0.0f));

	// physical properties
	setMass(1.0f);
	setCor(1.0f);
}

RigidBody::~RigidBody()
{
}



// Calculates the inertia matrix
glm::mat3 RigidBody::calculateInertia()
{
	glm::mat3 I = glm::mat3(0.0f);
	float w = getScale()[0][0];
	float h = getScale()[1][1];
	float d = getScale()[2][2];
	I[0][0] = getMass() * (h * h + d * d) / 12.0f;
	I[1][1] = getMass() * (w * w + d * d) / 12.0f;
	I[2][2] = getMass() * (w * w + h * h) / 12.0f;
	return I;
}



// Overrides setMass() to also set the inverse inertia
void RigidBody::setMass(const float & m)
{
	Body::setMass(m);
	setInvInertia(glm::inverse(calculateInertia()));
}



// Overrides scale() to also set the inverse inertia
void RigidBody::scale(const glm::vec3 & vect)
{
	Body::scale(vect);
	setInvInertia(glm::inverse(calculateInertia()));
}