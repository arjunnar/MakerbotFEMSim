#include "ElementMesh.h"


ElementMesh::ElementMesh(void)
{
	
}

void ElementMesh::buildStiffnessIndexHelper()
{
	originalToNewIndexes = std::vector<int>(3*coords.size());
	int numVerticesNotFixed = 0;
	for (int ii = 0; ii < coords.size(); ++ii)
	{
		if (fixedVertexIndexes.count(ii) == 0)
		{
			// vertex isn't fixed
			originalToNewIndexes[3*ii] = 3*numVerticesNotFixed;
			originalToNewIndexes[3*ii+1] = 3*numVerticesNotFixed+1;
			originalToNewIndexes[3*ii+2] = 3*numVerticesNotFixed+2;
			++numVerticesNotFixed;
		}

		else 
		{
			// vertex is fixed 
			originalToNewIndexes[3*ii] = -1;
			originalToNewIndexes[3*ii+1] = -1;
			originalToNewIndexes[3*ii+2] = -1;
		}
	}
}

int ElementMesh::getNumNonFixedVertices()
{
	return coords.size() - fixedVertexIndexes.size();
}


