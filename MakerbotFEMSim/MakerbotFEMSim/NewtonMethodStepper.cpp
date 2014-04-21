#include "NewtonMethodStepper.h"


NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh)
{
	this->mesh = mesh;
	Eigen::Vector3f force (1,-1,0);
	mesh->externalForcesPerVertex.push_back(force);
	for (int i = 0; i < mesh->externalForcesPerVertex.size(); ++i)
	{
		totalExternalForce += mesh->externalForcesPerVertex[i];
	}
}


void NewtonMethodStepper::step()
{
	Eigen::VectorXf totalForceVector(3*mesh->coords.size());

	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce;
	}

	for (int elementI = 0; elementI < mesh->elements.size(); ++elementI)
	{
		HexElement * elem = (HexElement*) mesh->elements[elementI];
		for (int ii = 0; ii < elem->vertices.size(); ++ii)
		{
			Eigen::Vector3f forceOnVertex = elem->getForce(ii);
			int sharedCoordIndex = elem->vertices[ii];
			if (sharedCoordIndex == 0 || sharedCoordIndex == 1 || sharedCoordIndex == 2 || sharedCoordIndex == 3)
			{
				continue;
			}

			totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) = totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) + forceOnVertex;
		}
	}

	// gradient descent 
	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		Eigen::Vector3f force = totalForceVector.block(3*sharedCoordI, 0, 3, 1);

		if (sharedCoordI == 0 || sharedCoordI == 1 || sharedCoordI == 2 || sharedCoordI == 3)
		{
			continue;
		}

		mesh->coords[sharedCoordI] += 0.0001*force;
	}
}