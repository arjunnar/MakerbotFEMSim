#include "NewtonMethodStepper.h"


NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh)
{
	this->mesh = mesh;
	totalExternalForce = Eigen::Vector3f::Zero();
	Eigen::Vector3f force(100,-100,0);
	mesh->externalForcesPerVertex.push_back(force);
	for (int i = 0; i < mesh->externalForcesPerVertex.size(); ++i)
	{
		totalExternalForce += mesh->externalForcesPerVertex[i];
	}

	//std::cout << "Total external force: " << totalExternalForce << std::endl;
}


void NewtonMethodStepper::step()
{
	Eigen::VectorXf totalForceVector(3*mesh->coords.size());

	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		if (sharedCoordI == 0 || sharedCoordI == 1 || sharedCoordI == 2 || sharedCoordI == 3)
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = Eigen::Vector3f::Zero();
		}
		
		else 
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce;
		}
	}

	std::cout << "Initial total force vector: " << totalForceVector << std::endl;

	for (int elementI = 0; elementI < mesh->elements.size(); ++elementI)
	{
		HexElement * elem = (HexElement*) mesh->elements[elementI];
		for (int ii = 0; ii < elem->vertices.size(); ++ii)
		{
			int sharedCoordIndex = elem->vertices[ii];
			if (sharedCoordIndex == 0 || sharedCoordIndex == 1 || sharedCoordIndex == 2 || sharedCoordIndex == 3)
			{
				continue;
			}

			Eigen::Vector3f forceOnVertex = elem->getForce(ii);
			std::cout << "Force on vertex " << ii << ": " << forceOnVertex << std::endl;
			
			totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) = totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) + forceOnVertex;
		}
	}

	std::cout << totalForceVector << std::endl;

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