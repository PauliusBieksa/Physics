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


// Use vel for wind speed
glm::vec3 SurfaceDrag::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	float ro = 1.2f;
	float cd = 1.5f;
	std::vector<glm::vec3> a0v;
	std::vector<glm::vec3> vs;
	std::vector<glm::vec3> n;
	switch (m_attachedTo.size())
	{
	case 2:
		a0v.push_back(glm::cross(m_attachedTo[0]->getPos() - pos, m_attachedTo[1]->getPos() - pos) * 0.5f);
		vs.push_back((m_self->getVel() + m_attachedTo[0]->getVel() + m_attachedTo[1]->getVel()) / 3.0f - vel);
		break;
	case 3:
		a0v.push_back(glm::cross(m_attachedTo[0]->getPos() - pos, m_attachedTo[1]->getPos() - pos) * 0.5f);
		a0v.push_back(glm::cross(m_attachedTo[1]->getPos() - pos, m_attachedTo[2]->getPos() - pos) * 0.5f);
		vs.push_back((m_self->getVel() + m_attachedTo[0]->getVel() + m_attachedTo[1]->getVel()) / 3.0f - vel);
		vs.push_back((m_self->getVel() + m_attachedTo[1]->getVel() + m_attachedTo[2]->getVel()) / 3.0f - vel);
		break;
	case 4:
		a0v.push_back(glm::cross(m_attachedTo[0]->getPos() - pos, m_attachedTo[1]->getPos() - pos) * 0.5f);
		a0v.push_back(glm::cross(m_attachedTo[1]->getPos() - pos, m_attachedTo[2]->getPos() - pos) * 0.5f);
		a0v.push_back(glm::cross(m_attachedTo[2]->getPos() - pos, m_attachedTo[3]->getPos() - pos) * 0.5f);
		a0v.push_back(glm::cross(m_attachedTo[3]->getPos() - pos, m_attachedTo[0]->getPos() - pos) * 0.5f);
		vs.push_back((m_self->getVel() + m_attachedTo[0]->getVel() + m_attachedTo[1]->getVel()) / 3.0f - vel);
		vs.push_back((m_self->getVel() + m_attachedTo[1]->getVel() + m_attachedTo[2]->getVel()) / 3.0f - vel);
		vs.push_back((m_self->getVel() + m_attachedTo[2]->getVel() + m_attachedTo[3]->getVel()) / 3.0f - vel);
		vs.push_back((m_self->getVel() + m_attachedTo[3]->getVel() + m_attachedTo[0]->getVel()) / 3.0f - vel);
		break;
	default:
		return glm::vec3(0.0f);
	}
	for (int i = 0; i < a0v.size(); i++)
		if (glm::length(a0v[i]) == 0)
			n.push_back(glm::vec3(0.0f));
		else
			n.push_back(a0v[i] / glm::length(a0v[i]));

	glm::vec3 f;
	for (int i = 0; i < a0v.size(); i++)
		if (glm::length(vs[i]) != 0.0f)
			f += n[i] * 0.5f * ro * glm::length2(vs[i]) * cd * glm::length(a0v[i]) * glm::dot(n[i], vs[i] / glm::length(vs[i]));
	return f / (float)a0v.size();
}
