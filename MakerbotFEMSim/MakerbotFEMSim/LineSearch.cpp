#include "LineSearch.h"

void LineSearch::advanceMesh(ElementMesh * mesh, Eigen::VectorXf &deltaX)
{
	Quadrature quadrature;
	float stepSize = 1.0f;

	while (true)
	{
		std::cout << "line search step size: " << stepSize << std::endl;
		// try advancing mesh 
		addDeltaX(mesh, deltaX, stepSize);

		// assess stability of system 
		bool stable = true;
		for (int eleI = 0; eleI < mesh->elements.size(); ++eleI)
		{
			if (!stable)
			{
				break;
			}
			
			HexElement * element = (HexElement*) mesh->elements[eleI];
			std::vector<Eigen::Vector3f> elemDeformedCoords; 

			for (int ii = 0; ii < element->vertices.size(); ++ii)
			{
				elemDeformedCoords.push_back(mesh->coords[element->vertices[ii]]);
			}

			for (int jj = 0; jj < NVERT; ++jj)
			{
				Eigen::Matrix3f defGradAtQuadPoint = element->defGradAtQuadPoint(elemDeformedCoords, quadrature.gaussCubePoints[jj]);
				float J = defGradAtQuadPoint.determinant();
				if (J < 0)
				{
					stable = false;
					break;
				}
			}
		}
		
		// if the system is stable, then we're done
		if (stable)
		{
			break;
		}

		// if the system is not stable then backtrack and reduce the step size
		else
		{
			addDeltaX(mesh, deltaX, -1.0*stepSize);
			stepSize /= 2.0;
		}
	}
}

void LineSearch::addDeltaX(ElementMesh * mesh, Eigen::VectorXf &deltaX, float stepSize)
{
	// advance mesh 
	int countNonFixed = 0;
	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		if (mesh->fixedVertexIndexes.count(sharedCoordI) > 0)
		{
			// vertex is fixed
			continue;
		}

		else 
		{
			mesh->coords[sharedCoordI] += stepSize * deltaX.block(3*countNonFixed, 0, 3, 1);
			++countNonFixed;
		}
	}
}

