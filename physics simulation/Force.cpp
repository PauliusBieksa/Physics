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
	// complete . Should return the acceleration resulting from gravity
	return glm::vec3(0.0f, 9.8f, 0.0f);
}

/*
** DRAG
*/
glm::vec3 Drag::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	// complete . Should return the acceleration resulting from aerodynamic drag
	float air_density = 1.2f;
	return (vel * -0.5f * 0.01f * 1.2f * air_density * vel.length()) / mass;
}
