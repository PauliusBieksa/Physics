// Math constants
# define _USE_MATH_DEFINES
# include <cmath>
# include "RigidBody.h"


// default constructor : creates a particle represented by a default ( square ).
// Notes :
// - particle rotated so that it is orthogonal to the z axis .
// - scaled
// - no shader allocated by default to avoid creating a Shader object for each particle .
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

RigidBody ::~RigidBody()
{
}