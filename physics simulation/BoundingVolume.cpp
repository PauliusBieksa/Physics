#include "BoundingVolume.h"






// Calls ther required collision detection algorithm based on bounding volume types
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



// Checks for sphere-sphere collisions
bool BoundingVolume::sphereSphereCheck(SphereCollider other)
{
	float d2 = m_sphere.getRadius() + other.getRadius();
	d2 *= d2;
	if (glm::length2(m_sphere.getPos() - other.getPos()) > d2)
		return false;
	return true;
}



// Checks for OBB-OBB collisions
bool BoundingVolume::OBBOBBCheck(OBBCollider other)
{
	float ra, rb;
	glm::mat3 absR, R;
	glm::vec3 hla, hlb;

	hla = m_OBB.getHalfLengths();
	hlb = other.getHalfLengths();


	// Compute the actually not rotation matrix expresing B in A's coordinate frame
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(m_OBB.getRot()[i], other.getRot()[j]);

	// Translation vector
	glm::vec3 t = other.getPos() - m_OBB.getPos();
	// Bring t into a's coordinate frame
	t = glm::vec3(glm::dot(t, m_OBB.getRot()[0]), glm::dot(t, m_OBB.getRot()[1]), glm::dot(t, m_OBB.getRot()[2]));

	// Compute common subexpresions (+ epsilon for parallel edges)
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			absR[i][j] = abs(R[i][j]) + 0.00001f;


	// Test A's local axes
	for (int i = 0; i < 3; i++)
	{
		ra = hla[i];
		rb = hlb[0] * absR[i][0] + hlb[1] * absR[i][1] + hlb[2] * absR[i][2];
		if (abs(t[i]) > ra + rb)
			return false;
	}

	// Test for B's local axes
	for (int i = 0; i < 3; i++)
	{
		ra = hla[0] * absR[0][i] + hla[1] * absR[1][i] + hla[2] * absR[2][i];
		rb = hlb[i];
		if (abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb)
			return false;
	}

	// A0 x B0
	ra = hla[1] * absR[2][0] + hla[2] * absR[1][0];
	rb = hlb[1] * absR[0][2] + hlb[2] * absR[0][1];
	if (abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)
		return false;

	// A0 x B1
	ra = hla[1] * absR[2][1] + hla[2] * absR[1][1];
	rb = hlb[0] * absR[0][2] + hlb[2] * absR[0][0];
	if (abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb)
		return false;

	// A0 x B2
	ra = hla[1] * absR[2][2] + hla[2] * absR[1][2];
	rb = hlb[0] * absR[0][1] + hlb[1] * absR[0][0];
	if (abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb)
		return false;

	// A1 x B0
	ra = hla[0] * absR[2][0] + hla[2] * absR[0][0];
	rb = hlb[1] * absR[1][2] + hlb[2] * absR[1][1];
	if (abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb)
		return false;

	// A1 x B1
	ra = hla[0] * absR[2][1] + hla[2] * absR[0][1];
	rb = hlb[0] * absR[1][2] + hlb[2] * absR[1][0];
	if (abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)
		return false;

	// A1 x B2
	ra = hla[0] * absR[2][2] + hla[2] * absR[0][2];
	rb = hlb[0] * absR[1][1] + hlb[1] * absR[1][0];
	if (abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb)
		return false;

	// A2 x B0
	ra = hla[0] * absR[1][0] + hla[1] * absR[0][0];
	rb = hlb[1] * absR[2][2] + hlb[2] * absR[2][1];
	if (abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb)
		return false;

	// A2 x B1
	ra = hla[0] * absR[1][1] + hla[1] * absR[0][1];
	rb = hlb[0] * absR[2][2] + hlb[2] * absR[2][0];
	if (abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb)
		return false;

	// A2 x B2
	ra = hla[0] * absR[1][2] + hla[1] * absR[0][2];
	rb = hlb[0] * absR[2][1] + hlb[1] * absR[2][0];
	if (abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb)
		return false;


	return true;
}