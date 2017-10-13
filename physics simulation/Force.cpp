#include <iostream>
#include <cmath>
#include "Force.h"
#include "Body.h"
#include "glm/ext.hpp"

glm::vec3 Force::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	return glm::vec3(0.0f);

}

/*
** GRAVITY
*/
glm::vec3 Gravity::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	return glm::vec3(0.0f, -9.8f, 0.0f) * mass;
}

/*
** DRAG
*/
glm::vec3 Drag::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	float air_density = 1.2f;
	return (vel * -0.5f * 0.01f * 1.5f * air_density * length(vel));
}


// Hooke
glm::vec3 Hooke::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	glm::vec3 toAttached = m_attachedTo->getPos() - pos;
	float l = length(toAttached);
	if (l == 0.0f)
		toAttached[1] = -0.0000001;
	l = length(toAttached);
	glm::vec3 e = toAttached / l;
	float v1 = glm::dot(vel, e);
	float v2 = glm::dot(m_attachedTo->getVel(), e);
	return (m_ks * (l - m_rest) - m_kd * (v1 - v2)) * toAttached / l;
}
