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
	{
		toAttached[1] = -0.0000001;
		l = length(toAttached);
	}
	glm::vec3 e = toAttached / l;
	float v1 = glm::dot(vel, e);
	float v2 = glm::dot(m_attachedTo->getVel(), e);
	glm::vec3 fh = (m_ks * (l - m_rest) - m_kd * (v1 - v2)) * e;
	return fh;
}


// Use vel for wind speed
glm::vec3 SurfaceDrag::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	float ro = 1.2f;
	float cd = 1.5f;
	glm::vec3 a0v;
	glm::vec3 v;
	glm::vec3 n;

	if (m_iteration == 0)
	{
		// Calculates the surface drag
		m_iteration++;
		a0v = glm::cross(m_b2->getPos() - m_b1->getPos(), m_b3->getPos() - m_b1->getPos()) / 2.0f;
		v = (m_b1->getVel() + m_b2->getVel() + m_b3->getVel()) / 3.0f + *m_wind;
		if (glm::length2(a0v) == 0.0f || glm::length2(v) == 0.0f)
		{
			m_f = glm::vec3(0.0f);
			return m_f;
		}
		n = a0v / glm::length(a0v);
		m_f = -0.5f * ro * glm::length2(v) * cd * glm::length(a0v) * glm::dot(v, n) / glm::length(v) * n;
		return m_f;
	}
	else
	{
		// Returns the already calculated force
		if (m_iteration < 2)
			m_iteration++;
		else
			m_iteration = 0;
		return m_f;
	}
}
