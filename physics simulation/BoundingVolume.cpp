#include "BoundingVolume.h"






// Calls the required collision detection algorithm based on bounding volume types
std::pair<glm::mat2x3, float> BoundingVolume::collisionCheck(BoundingVolume other)
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

	// Returns nans if bounding volumes have incompatible types
	glm::mat2x3 plane = glm::mat2x3();
	plane *= nan("");
	std::pair<glm::mat2x3, float> result;
	result.first = plane;
	return result;
}



// Checks for sphere-sphere collisions and returns collision plane
std::pair<glm::mat2x3, float> BoundingVolume::sphereSphereCheck(SphereCollider other)
{
	// Stores plane of collision if collision happened / 0 - normal, 1 - point of collision
	glm::mat2x3 plane = glm::mat2x3();
	std::pair<glm::mat2x3, float> result;

	float d = m_sphere.getRadius() + other.getRadius();
	if (glm::length(m_sphere.getPos() - other.getPos()) > d)
	{
		plane[0] = (other.getPos() - m_sphere.getPos()) / 2.0f;
		plane[1] = m_sphere.getPos() + plane[0];
		plane[0] = plane[0] / glm::length(plane[0]);
		result.second = d - glm::length(m_sphere.getPos() - other.getPos());
		result.first = plane;
		return result;
	}
	// Return nans if no collision
	plane *= nan("");
	result.first = plane;
	return result;
}



// Checks for OBB-OBB collisions
std::pair<glm::mat2x3, float> BoundingVolume::OBBOBBCheck(OBBCollider other)
{
	float ra, rb;
	glm::mat3 absR, R;
	glm::vec3 hla, hlb;

	hla = m_OBB.getHalfLengths();
	hlb = other.getHalfLengths();


	// Compute the rotation matrix expresing B in A's coordinate frame
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
			absR[i][j] = abs(R[i][j]) + 0.000001f;


	// Stores plane of collision if collision happened / 0 - normal, 1 - point of collision
	glm::mat2x3 plane = glm::mat2x3();
	std::pair<glm::mat2x3, float> result;
	plane *= nan("");
	result.first = plane;
	float minD;
	int axisIndex = 0;
	float intersectionDistance;

	// Test A's local axes
	for (int i = 0; i < 3; i++)
	{
		ra = hla[i];
		rb = hlb[0] * absR[i][0] + hlb[1] * absR[i][1] + hlb[2] * absR[i][2];
		//if (abs(t[i]) > ra + rb)
		intersectionDistance = abs(t[i]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (i == 0 || minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = i;
		}
	}

	// Test for B's local axes
	for (int i = 0; i < 3; i++)
	{
		ra = hla[0] * absR[0][i] + hla[1] * absR[1][i] + hla[2] * absR[2][i];
		rb = hlb[i];
		//if (abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb)
		intersectionDistance = abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 3 + i;
		}
	}

	// If any of the axes of the two OBBs align skip all the product axes
	if (!(R[0][0] == 1.0f || R[1][1] == 1.0f || R[2][2] == 1.0f))
	{
		// A0 x B0
		ra = hla[1] * absR[2][0] + hla[2] * absR[1][0];
		rb = hlb[1] * absR[0][2] + hlb[2] * absR[0][1];
		//if (abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)
		intersectionDistance = abs(t[2] * R[1][0] - t[1] * R[2][0]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 6;
		}


		// A0 x B1
		ra = hla[1] * absR[2][1] + hla[2] * absR[1][1];
		rb = hlb[0] * absR[0][2] + hlb[2] * absR[0][0];
		//if (abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb)
		intersectionDistance = abs(t[2] * R[1][1] - t[1] * R[2][1]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 7;
		}


		// A0 x B2
		ra = hla[1] * absR[2][2] + hla[2] * absR[1][2];
		rb = hlb[0] * absR[0][1] + hlb[1] * absR[0][0];
		//if (abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb)
		intersectionDistance = abs(t[2] * R[1][2] - t[1] * R[2][2]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 8;
		}


		// A1 x B0
		ra = hla[0] * absR[2][0] + hla[2] * absR[0][0];
		rb = hlb[1] * absR[1][2] + hlb[2] * absR[1][1];
		//if (abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb)
		intersectionDistance = abs(t[0] * R[2][0] - t[2] * R[0][0]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 9;
		}


		// A1 x B1
		ra = hla[0] * absR[2][1] + hla[2] * absR[0][1];
		rb = hlb[0] * absR[1][2] + hlb[2] * absR[1][0];
		//if (abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)
		intersectionDistance = abs(t[0] * R[2][1] - t[2] * R[0][1]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 10;
		}


		// A1 x B2
		ra = hla[0] * absR[2][2] + hla[2] * absR[0][2];
		rb = hlb[0] * absR[1][1] + hlb[1] * absR[1][0];
		//if (abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb)
		intersectionDistance = abs(t[0] * R[2][2] - t[2] * R[0][2]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 11;
		}


		// A2 x B0
		ra = hla[0] * absR[1][0] + hla[1] * absR[0][0];
		rb = hlb[1] * absR[2][2] + hlb[2] * absR[2][1];
		//if (abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb)
		intersectionDistance = abs(t[1] * R[0][0] - t[0] * R[1][0]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 12;
		}


		// A2 x B1
		ra = hla[0] * absR[1][1] + hla[1] * absR[0][1];
		rb = hlb[0] * absR[2][2] + hlb[2] * absR[2][0];
		//if (abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb)
		intersectionDistance = abs(t[1] * R[0][1] - t[0] * R[1][1]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 13;
		}


		// A2 x B2
		ra = hla[0] * absR[1][2] + hla[1] * absR[0][2];
		rb = hlb[0] * absR[2][1] + hlb[1] * absR[2][0];
		//if (abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb)
		intersectionDistance = abs(t[1] * R[0][2] - t[0] * R[1][2]) - ra - rb;
		intersectionDistance = -intersectionDistance;
		if (intersectionDistance <= 0.0f)
			return result;
		// At this point a collision has been detected

		// Saves min distance and axis of min distance for collision plane info
		if (minD > intersectionDistance)
		{
			minD = intersectionDistance;
			axisIndex = 14;
		}
	}

	// Calculate possitions of all vertices of the bounding boxes (- position)
	glm::vec3 vertsA[8];
	vertsA[0] = m_OBB.getRot()[0] * hla[0] + m_OBB.getRot()[1] * hla[1] + m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	vertsA[1] = m_OBB.getRot()[0] * hla[0] + m_OBB.getRot()[1] * hla[1] - m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	vertsA[2] = m_OBB.getRot()[0] * hla[0] - m_OBB.getRot()[1] * hla[1] + m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	vertsA[3] = m_OBB.getRot()[0] * hla[0] - m_OBB.getRot()[1] * hla[1] - m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	vertsA[4] = -m_OBB.getRot()[0] * hla[0] + m_OBB.getRot()[1] * hla[1] + m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	vertsA[5] = -m_OBB.getRot()[0] * hla[0] + m_OBB.getRot()[1] * hla[1] - m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	vertsA[6] = -m_OBB.getRot()[0] * hla[0] - m_OBB.getRot()[1] * hla[1] + m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	vertsA[7] = -m_OBB.getRot()[0] * hla[0] - m_OBB.getRot()[1] * hla[1] - m_OBB.getRot()[2] * hla[2];// + m_OBB.getPos();
	glm::vec3 vertsB[8];
	vertsB[0] = other.getRot()[0] * hlb[0] + other.getRot()[1] * hlb[1] + other.getRot()[2] * hlb[2];// + other.getPos();
	vertsB[1] = other.getRot()[0] * hlb[0] + other.getRot()[1] * hlb[1] - other.getRot()[2] * hlb[2];// + other.getPos();
	vertsB[2] = other.getRot()[0] * hlb[0] - other.getRot()[1] * hlb[1] + other.getRot()[2] * hlb[2];// + other.getPos();
	vertsB[3] = other.getRot()[0] * hlb[0] - other.getRot()[1] * hlb[1] - other.getRot()[2] * hlb[2];// + other.getPos();
	vertsB[4] = -other.getRot()[0] * hlb[0] + other.getRot()[1] * hlb[1] + other.getRot()[2] * hlb[2];// + other.getPos();
	vertsB[5] = -other.getRot()[0] * hlb[0] + other.getRot()[1] * hlb[1] - other.getRot()[2] * hlb[2];// + other.getPos();
	vertsB[6] = -other.getRot()[0] * hlb[0] - other.getRot()[1] * hlb[1] + other.getRot()[2] * hlb[2];// + other.getPos();
	vertsB[7] = -other.getRot()[0] * hlb[0] - other.getRot()[1] * hlb[1] - other.getRot()[2] * hlb[2];// + other.getPos();

	int aMinY = 0;
	int aMinZ = 0;
	int bMinY = 0;
	int bMinZ = 0;
	int aMaxY = 0;
	int aMaxZ = 0;
	int bMaxY = 0;
	int bMaxZ = 0;
	glm::vec3 rotatedA[8];
	glm::vec3 rotatedB[8];
	int nOfCollisionsA = 0;
	glm::vec3 collisionPointA = glm::vec3(0.0f);
	int nOfCollisionsB = 0;
	glm::vec3 collisionPointB = glm::vec3(0.0f);
	t = other.getPos() - m_OBB.getPos();

	// Calculate the plane of collision, plane normal and point on the plane
	switch (axisIndex)
	{
	case 0:
		plane[0] = m_OBB.getRot()[0];
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(m_OBB.getRot()[0], m_OBB.getRot()[1], m_OBB.getRot()[2]);

		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (hla[0] < rb)
		{
			plane[1] = plane[0] * (hla[0] - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 1:
		plane[0] = m_OBB.getRot()[1];
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(m_OBB.getRot()[1], m_OBB.getRot()[2], m_OBB.getRot()[0]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (hla[1] < rb)
		{
			plane[1] = plane[0] * (hla[1] - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 2:
		plane[0] = m_OBB.getRot()[2];
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(m_OBB.getRot()[2], m_OBB.getRot()[0], m_OBB.getRot()[1]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (hla[2] < rb)
			plane[1] = plane[0] * (hla[2] - minD / 2.0f);
		else
			plane[1] = plane[0] * (rb - minD / 2.0f);
		plane[1] += m_OBB.getPos();
		break;
	case 3:
		plane[0] = other.getRot()[0];
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(other.getRot()[0], other.getRot()[1], other.getRot()[2]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[0][0] + hla[1] * absR[1][0] + hla[2] * absR[2][0];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 4:
		plane[0] = other.getRot()[1];
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(other.getRot()[1], other.getRot()[2], other.getRot()[0]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[0][1] + hla[1] * absR[1][1] + hla[2] * absR[2][1];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 5:
		plane[0] = other.getRot()[2];
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(other.getRot()[2], other.getRot()[0], other.getRot()[1]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[0][2] + hla[1] * absR[1][2] + hla[2] * absR[2][2];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 6:
		plane[0] = glm::cross(m_OBB.getRot()[0], other.getRot()[0]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[0]), m_OBB.getRot()[0]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[1] * absR[2][0] + hla[2] * absR[1][0];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 7:
		plane[0] = glm::cross(m_OBB.getRot()[0], other.getRot()[1]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[0]), m_OBB.getRot()[0]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[1] * absR[2][1] + hla[2] * absR[1][1];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 8:
		plane[0] = glm::cross(m_OBB.getRot()[0], other.getRot()[2]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[0]), m_OBB.getRot()[0]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[1] * absR[2][2] + hla[2] * absR[1][2];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 9:
		plane[0] = glm::cross(m_OBB.getRot()[1], other.getRot()[0]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[1]), m_OBB.getRot()[1]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[2][0] + hla[2] * absR[0][0];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 10:
		plane[0] = glm::cross(m_OBB.getRot()[1], other.getRot()[1]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[1]), m_OBB.getRot()[1]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[2][1] + hla[2] * absR[0][1];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 11:
		plane[0] = glm::cross(m_OBB.getRot()[1], other.getRot()[2]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[1]), m_OBB.getRot()[1]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[2][2] + hla[2] * absR[0][2];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 12:
		plane[0] = glm::cross(m_OBB.getRot()[2], other.getRot()[0]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[2]), m_OBB.getRot()[2]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[1][0] + hla[1] * absR[0][0];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 13:
		plane[0] = glm::cross(m_OBB.getRot()[2], other.getRot()[1]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[2]), m_OBB.getRot()[2]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = hla[0] * absR[1][1] + hla[1] * absR[0][1];

		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	case 14:
		plane[0] = glm::cross(m_OBB.getRot()[2], other.getRot()[2]);
		plane[0] /= glm::length(plane[0]);
		if (glm::dot(t, plane[0]) < 0.0f)/////////////////////////////////////////////////////////////////////
			plane[0] *= -1.0f;
		R = glm::mat3(plane[0], glm::cross(plane[0], m_OBB.getRot()[2]), m_OBB.getRot()[2]);
		for (int i = 0; i < 3; i++)
			R[i] /= glm::length(R[i]);
		// Get all the verteces rotated to R[n, yn, zn] coordinate frame centering on a point on the collision plane
		for (int i = 0; i < 8; i++)
		{
			// v * R[n, yn, zn]
			rotatedA[i] = R * vertsA[i] + m_OBB.getPos();
			rotatedB[i] = R * vertsB[i] + other.getPos();
			// Get limits of intersection in R[n, yn, zn] coordinate frame
			// y
			if (rotatedA[i][1] < rotatedA[aMinY][1])
				aMinY = i;
			else if (rotatedA[i][1] > rotatedA[aMaxY][1])
				aMaxY = i;
			if (rotatedB[i][1] < rotatedB[bMinY][1])
				bMinY = i;
			else if (rotatedB[i][1] > rotatedB[bMaxY][1])
				bMaxY = i;
			// z
			if (rotatedA[i][2] < rotatedA[aMinZ][2])
				aMinZ = i;
			else if (rotatedA[i][2] > rotatedA[aMaxZ][2])
				aMaxZ = i;
			if (rotatedB[i][2] < rotatedB[bMinZ][2])
				bMinZ = i;
			else if (rotatedB[i][2] > rotatedB[bMaxZ][2])
				bMaxZ = i;
		}
		ra = rotatedA[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedA[i][0] > ra)
				ra = rotatedA[i][0];
		}
		ra = ra - m_OBB.getPos()[0];
		rb = rotatedB[0][0];
		for (int i = 1; i < 8; i++)
		{
			if (rotatedB[i][0] > rb)
				rb = rotatedB[i][0];
		}
		rb = rb - other.getPos()[0];
		if (ra < rb)
		{
			plane[1] = plane[0] * (ra - minD / 2.0f);
			plane[1] += m_OBB.getPos();
		}
		else
		{
			plane[1] = -plane[0] * (rb - minD / 2.0f);
			plane[1] += other.getPos();
		}
		break;
	}

	// Get all the points beyond the collision plane by the minimum intersection distance and within 
	for (int i = 0; i < 8; i++)
	{
		// If dot product is positive, point is in front of the plane 
		float adsfasfa = glm::dot(plane[0], vertsA[i] + m_OBB.getPos() - plane[1]);//////////////////////////////////
		if (glm::dot(plane[0], vertsA[i] + m_OBB.getPos() - plane[1]) >= 0.0f
			&& ((rotatedA[i][1] <= rotatedB[bMaxY][1] && rotatedA[i][1] >= rotatedB[bMinY][1])
				|| (rotatedA[i][2] <= rotatedB[bMaxZ][2] && rotatedA[i][2] >= rotatedB[bMinZ][2])))
		{
			nOfCollisionsA++;
			collisionPointA += vertsA[i] + m_OBB.getPos();
		}
		// If dot product is negative, point is behind the plane 
		if (glm::dot(plane[0], vertsB[i] + other.getPos() - plane[1]) <= 0.0f
			&& ((rotatedB[i][1] <= rotatedA[aMaxY][1] && rotatedB[i][1] >= rotatedA[aMinY][1])
				|| (rotatedB[i][2] <= rotatedA[aMaxZ][2] && rotatedB[i][2] >= rotatedA[aMinZ][2])))
		{
			nOfCollisionsB++;
			collisionPointB += vertsB[i] + other.getPos();
		}
	}
	if (nOfCollisionsA == nOfCollisionsB && nOfCollisionsA == 0)
	{
		result.first *= nan("");
		return result;
	}
	if (nOfCollisionsA == nOfCollisionsB)
	{
		collisionPointA += collisionPointB;
		collisionPointA /= (float)(nOfCollisionsA + nOfCollisionsB);
		plane[1] = collisionPointA;
	}
	else if (nOfCollisionsA != 0 && nOfCollisionsA < nOfCollisionsB)
	{
		collisionPointA /= (float)nOfCollisionsA;
		plane[1] = collisionPointA;
	}
	else if (nOfCollisionsB != 0 && nOfCollisionsA > nOfCollisionsB)
	{
		collisionPointB /= (float)nOfCollisionsB;
		plane[1] = collisionPointB;
	}
	else if (nOfCollisionsA < nOfCollisionsB)
	{
		collisionPointB /= (float)nOfCollisionsB;
		plane[1] = collisionPointB;
	}
	else if (nOfCollisionsA > nOfCollisionsB)
	{
		collisionPointA /= (float)nOfCollisionsA;
		plane[1] = collisionPointA;
	}

	result.first = plane;
	result.second = minD;
	return result;
}