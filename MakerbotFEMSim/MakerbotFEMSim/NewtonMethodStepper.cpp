#include "NewtonMethodStepper.h"


NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh)
{
	this->mesh = mesh;
	totalExternalForce = Eigen::Vector3f::Zero();
	Eigen::Vector3f force(00,00,0);
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
		if (mesh->sharedIndexBase.count(sharedCoordI) > 0)
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = Eigen::Vector3f::Zero();
		}
		
		else 
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce;
		}
	}

	//std::cout << "Initial total force vector: " << totalForceVector << std::endl;

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
			if (mesh->sharedIndexBase.count(sharedCoordIndex) > 0)
			{
				continue;
			}

			Eigen::Vector3f forceOnVertex = elem->getForce(elemDeformedCoords, ii);
		//	std::cout << "Force on vertex " << ii << ": " << forceOnVertex << std::endl;
			
			totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) = totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) + forceOnVertex;
		}
	}

	//std::cout << totalForceVector << std::endl;

	// gradient descent 
	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		Eigen::Vector3f force = totalForceVector.block(3*sharedCoordI, 0, 3, 1);

		if (mesh->sharedIndexBase.count(sharedCoordI) > 0)
		{
			continue;
		}

		mesh->coords[sharedCoordI] += 0.0001*force;
	}
}