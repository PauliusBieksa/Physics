#include "Body.h"

Body::Body()
{
}

Body ::~Body()
{
}

/* TRANSFORMATION METHODS */
void Body::translate(const glm::vec3 & vect)
{
	m_pos = m_pos + vect;
	m_mesh.translate(vect);
}

void Body::rotate(float angle, const glm::vec3 & vect)
{
	m_mesh.rotate(angle, vect);
}

void Body::scale(const glm::vec3 & vect)
{
	m_mesh.scale(vect);
}




glm::vec3 Body::applyForces(glm::vec3 pos, glm::vec3 vel)
{
	glm::vec3 fAccumulator = glm::vec3(0.0f);
	for (auto &f : m_forces)
	{
		fAccumulator += f->apply(getMass(), pos, vel);
	}
	glm::vec3 acc = fAccumulator / m_mass;
	// Safeguard against unresonable accelerations
	if (glm::length(acc) > 100.0f)
		return acc / glm::length(acc) * 100.0f;
	return acc;
}
