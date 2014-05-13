#include "ElementMesh.h"


ElementMesh::ElementMesh(void)
{
	numNonFixedVertices = -1;
}

int ElementMesh::getNumNonFixedVertices()
{
	if (numNonFixedVertices = -1)
	{
		numNonFixedVertices = 0;
		for (int sharedCoordI = 0; sharedCoordI < coords.size(); ++sharedCoordI)
		{
			if (sharedIndexBase.count(sharedCoordI) == 0)
			{
				++numNonFixedVertices;
			}
		}
	}

	return numNonFixedVertices;
}


