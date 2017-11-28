#pragma once
#include <glm/glm.hpp>
#include <GL/glew.h>
#include "glm/ext.hpp"




// Sphere collider
class SphereCollider
{
public:
	SphereCollider() {}///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SphereCollider(glm::vec3 position, float radius) { m_pos = position; m_r = radius; }



	glm::vec3 getPos() { return m_pos; }
	float getRadius() { return m_r; }

private:
	glm::vec3 m_pos;
	float m_r;
};



// Sphere collider
class OBBCollider
{
public:
	OBBCollider() {}///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	OBBCollider(glm::vec3 position, glm::mat3 rotation_matrix, glm::vec3 halfLengths) { m_pos = position; m_rot = rotation_matrix; m_half_lengths = halfLengths; }

	glm::vec3 getPos() { return m_pos; }
	glm::mat3 getRot() { return m_rot; }
	glm::vec3 getHalfLengths() { return m_half_lengths; }

private:
	glm::vec3 m_pos;
	glm::mat3 m_rot;
	glm::vec3 m_half_lengths;
};




class BoundingVolume
{
public:
	enum ColliderType
	{
		SPHERE,
		OBB
	};

	// Sphere constructor
	BoundingVolume(glm::vec3 position, float radius) { m_sphere = SphereCollider(position, radius); m_type = SPHERE; }
	// OBB constructor
	BoundingVolume(glm::vec3 position, glm::mat3 rotation_matrix, glm::vec3 halfLengths) { m_OBB = OBBCollider(position, rotation_matrix, halfLengths); m_type = OBB; }

	// Checks for collisions between two bounding volumes
	bool collisionCheck(BoundingVolume other);


	ColliderType getColliderType() { return m_type; }
	SphereCollider getSphereCollider() { return m_sphere; }
	OBBCollider getOBBCollider() { return m_OBB; }

private:
	// Checks for sphere-sphere collisions
	bool sphereSphereCheck(SphereCollider other);
	// Checks for OBB-OBB collisions
	bool OBBOBBCheck(OBBCollider other);


	SphereCollider m_sphere;
	OBBCollider m_OBB;
	ColliderType m_type;
};
