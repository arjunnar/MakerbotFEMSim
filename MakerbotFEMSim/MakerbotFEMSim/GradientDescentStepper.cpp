#include "GradientDescentStepper.h"


GradientDescentStepper::GradientDescentStepper(ElementMesh * mesh) : BaseStepper(mesh)
{
	totalExternalForce = Eigen::Vector3f::Zero();
	Eigen::Vector3f force(.02,0,0);
	//	Eigen::Vector3f force(1.0f,0,0);
	mesh->externalForcesPerVertex.push_back(force);
	for (int i = 0; i < mesh->externalForcesPerVertex.size(); ++i)
	{
		totalExternalForce += mesh->externalForcesPerVertex[i];
	}
}


void GradientDescentStepper::step()
{
	Eigen::VectorXf totalForceVector(3*mesh->coords.size());

	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		if (mesh->fixedVertexIndexes.count(sharedCoordI) > 0 || sharedCoordI < 72)
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = Eigen::Vector3f::Zero();
		}
		
		else 
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce;
		}
	}

	for (int elementI = 0; elementI < mesh->elements.size(); ++elementI)
	{
		HexElement * elem = (HexElement*) mesh->elements[elementI];
		std::vector<Eigen::Vector3f> elemDeformedCoords; 

		for (int ii = 0; ii < elem->vertices.size(); ++ii)
		{
			elemDeformedCoords.push_back(mesh->coords[elem->vertices[ii]]);
		}
		
		for (int ii = 0; ii < elem->vertices.size(); ++ii)
		{
			int sharedCoordIndex = elem->vertices[ii];
			if (mesh->fixedVertexIndexes.count(sharedCoordIndex) > 0)
			{
				continue;
			}
			Eigen::Vector3f forceOnVertex = elem->getForce(elemDeformedCoords, ii);

			totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) = totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) + forceOnVertex;
		}
	}

	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		Eigen::Vector3f force = totalForceVector.block(3*sharedCoordI, 0, 3, 1);

		if (mesh->fixedVertexIndexes.count(sharedCoordI) > 0)
		{
			continue;
		}

		mesh->coords[sharedCoordI] += 0.01*force;
	}
}