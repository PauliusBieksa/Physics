#pragma once
#include <glm/glm.hpp>
#include <GL/glew.h>
#include "glm/ext.hpp"
#include "RigidBody.h"




// Sphere collider
class SphereCollider
{
public:
	SphereCollider() {}///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	SphereCollider(glm::vec3 position, float radius) { m_pos = position; m_r = radius; }


	void update(glm::vec3 position) { m_pos = position; }
	void update(glm::vec3 position, float radius) { m_pos = position; m_r = radius; }


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


	void update(glm::vec3 position, glm::mat3 rotationMatrix) { m_pos = position; m_rot = rotationMatrix; }
	void update(glm::vec3 position, glm::mat3 rotationMatrix, float x_scale, float y_scale, float z_scale)
	{
		m_pos = position;
		m_rot = rotationMatrix;
		m_half_lengths = glm::vec3(x_scale, y_scale, z_scale) / 2.0f;
	}


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

	// Default constructor
	BoundingVolume() { m_sphere = SphereCollider(glm::vec3(), 0.01f); m_type = SPHERE; }
	// Sphere constructor
	BoundingVolume(glm::vec3 position, float radius) { m_sphere = SphereCollider(position, radius); m_type = SPHERE; }
	// OBB constructor
	BoundingVolume(glm::vec3 position, glm::mat3 rotation_matrix, glm::vec3 halfLengths) { m_OBB = OBBCollider(position, rotation_matrix, halfLengths); m_type = OBB; }

	// Checks for collisions between two bounding volumes
	glm::mat2x3 collisionCheck(BoundingVolume other);

	void updateSphere(glm::vec3 position) { m_sphere.update(position); }
	void updateOBB(glm::vec3 position, glm::mat3 rotationMatrix) { m_OBB.update(position, rotationMatrix); }


	const ColliderType& getColliderType() { return m_type; }
	const SphereCollider& getSphereCollider() { return m_sphere; }
	const OBBCollider& getOBBCollider() { return m_OBB; }

private:
	// Checks for sphere-sphere collisions
	glm::mat2x3 sphereSphereCheck(SphereCollider other);
	// Checks for OBB-OBB collisions
	glm::mat2x3 OBBOBBCheck(OBBCollider other);


	SphereCollider m_sphere;
	OBBCollider m_OBB;
	ColliderType m_type;
};
