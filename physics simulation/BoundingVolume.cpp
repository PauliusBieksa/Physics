#include "BoundingVolume.h"





bool BoundingVolume::collisionCheck(BoundingVolume other)
{
	switch (m_type)
	{
	case SPHERE:
		switch (other.getColliderType)
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
		switch (other.getColliderType)
		{
		// OBB-sphere
		case SPHERE:
			break;
		// OBB-OBB
		case OBB:
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
	if (glm::length2(m_sphere.getPos() - other.getPos()) < d2)
		return true;
	return false;
}



bool BoundingVolume::OBBOBBCheck(OBBCollider other)
{
	return false;
}