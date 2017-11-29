#include "BoundingVolume.h"







bool BoundingVolume::collisionCheck(BoundingVolume other)
{
	switch (m_type)
	{
	case SPHERE:
		switch (other.getColliderType())
		{
		// Sphere-sphere
		case SPHERE:
			return sphereSphereCheck(other.getSphereCollider());
			break;
		// Sphere-OBB
		case OBB:
			break;
		}
		break;
	case OBB:
		switch (other.getColliderType())
		{
		// OBB-sphere
		case SPHERE:
			break;
		// OBB-OBB
		case OBB:
			return OBBOBBCheck(other.getOBBCollider());
			break;
		}
		break;
	}
	return false;
}




bool BoundingVolume::sphereSphereCheck(SphereCollider other)
{
	float d2 = m_sphere.getRadius() + other.getRadius();
	d2 *= d2;
	if (glm::length2(m_sphere.getPos() - other.getPos()) > d2)
		return false;
	return true;
}



bool BoundingVolume::OBBOBBCheck(OBBCollider other)
{
	glm::vec3 tmp;

	float ra, rb;
	glm::mat3 absR, R;

	// Rotation matrix of A
//	R = m_OBB.getRot();

	// Compute the rotation matrix expresing B in A's coordinate frame
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(m_OBB.getRot()[i], other.getRot()[j]);

	// Translation vector
	glm::vec3 t = other.getPos() - m_OBB.getPos();
	// Store scalar projections on local axes of A in the t vector
	t = glm::vec3(glm::dot(t, R[0]), glm::dot(t, R[1]), glm::dot(t, R[2]));

	// Compute common subexpresions (+ epsilon for parallel edges)
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			absR[i][j] = abs(R[i][j]) + 0.00001f;


	// Test A's local axes
	for (int i = 0; i < 3; i++)
	{
		ra = m_OBB.getHalfLengths()[i];
		tmp = other.getHalfLengths();
		rb = tmp[0] * absR[i][0] + tmp[1] * absR[i][1] + tmp[2] * absR[i][2];
		if (abs(t[i]) > ra + rb)
			return false;
	}

	// Test for B's local axes
	for (int i = 0; i < 3; i++)
	{
		tmp = m_OBB.getHalfLengths();
		ra = tmp[0] * absR[i][0] + tmp[1] * absR[i][1] + tmp[2] * absR[i][2];
		rb = other.getHalfLengths()[i];
		if (abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb)
			return false;
	}

	// A0 x B0
	ra = m_OBB.getHalfLengths()[1] * absR[2][0] + m_OBB.getHalfLengths()[2] * absR[1][0];
	rb = other.getHalfLengths()[1] * absR[0][2] + other.getHalfLengths()[2] * absR[0][1];
	if (abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)
		return false;

	// A0 x B1
	ra = m_OBB.getHalfLengths()[1] * absR[2][1] + m_OBB.getHalfLengths()[2] * absR[1][1];
	rb = other.getHalfLengths()[0] * absR[0][2] + other.getHalfLengths()[2] * absR[0][0];
	if (abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb)
		return false;

	// A0 x B2
	ra = m_OBB.getHalfLengths()[1] * absR[2][2] + m_OBB.getHalfLengths()[2] * absR[1][2];
	rb = other.getHalfLengths()[0] * absR[0][1] + other.getHalfLengths()[1] * absR[0][0];
	if (abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb)
		return false;

	// A1 x B0
	ra = m_OBB.getHalfLengths()[0] * absR[2][0] + m_OBB.getHalfLengths()[2] * absR[0][0];
	rb = other.getHalfLengths()[1] * absR[1][2] + other.getHalfLengths()[2] * absR[1][1];
	if (abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb)
		return false;

	// A1 x B1
	ra = m_OBB.getHalfLengths()[0] * absR[2][1] + m_OBB.getHalfLengths()[2] * absR[0][1];
	rb = other.getHalfLengths()[0] * absR[1][2] + other.getHalfLengths()[2] * absR[1][0];
	if (abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)
		return false;

	// A1 x B2
	ra = m_OBB.getHalfLengths()[0] * absR[2][2] + m_OBB.getHalfLengths()[2] * absR[0][2];
	rb = other.getHalfLengths()[0] * absR[1][1] + other.getHalfLengths()[1] * absR[1][0];
	if (abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb)
		return false;

	// A2 x B0
	ra = m_OBB.getHalfLengths()[0] * absR[1][0] + m_OBB.getHalfLengths()[1] * absR[0][0];
	rb = other.getHalfLengths()[1] * absR[2][2] + other.getHalfLengths()[2] * absR[2][1];
	if (abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb)
		return false;

	// A2 x B1
	ra = m_OBB.getHalfLengths()[0] * absR[1][1] + m_OBB.getHalfLengths()[1] * absR[0][1];
	rb = other.getHalfLengths()[0] * absR[2][2] + other.getHalfLengths()[2] * absR[2][0];
	if (abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb)
		return false;

	// A2 x B2
	ra = m_OBB.getHalfLengths()[0] * absR[1][2] + m_OBB.getHalfLengths()[1] * absR[0][2];
	rb = other.getHalfLengths()[0] * absR[2][1] + other.getHalfLengths()[1] * absR[2][0];
	if (abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb)
		return false;


	return true;
}