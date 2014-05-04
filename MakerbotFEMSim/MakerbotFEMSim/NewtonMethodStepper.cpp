#include "NewtonMethodStepper.h"


NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh) : BaseStepper(mesh)
{
	totalExternalForce = Eigen::Vector3f::Zero();
	Eigen::Vector3f force(10.0f,0,0);
	mesh->externalForcesPerVertex.push_back(force);
	for (int i = 0; i < mesh->externalForcesPerVertex.size(); ++i)
	{
		totalExternalForce += mesh->externalForcesPerVertex[i];
	}
}


void NewtonMethodStepper::step()
{
	std::cout << "Taking Newton's Method step" << std::endl;
	float stepSize = 1.0f;

	// TEST WITH ONE ELEMENT NOW
	HexElement * elem = (HexElement*) mesh->elements[0];

	// K is computed with element ordering, so use element ordering for total force as well (don't use global shared coord ordering)
	Eigen::VectorXf totalForceVector(3*8); 

	for (int ii = 0; ii < 8; ++ii)
	{
		// fix base of single element
		if (elem->vertices[ii] < 4)
		{
			totalForceVector.block(3*ii, 0, 3, 1) = Eigen::Vector3f::Zero();
		}
		
		else 
		{
			totalForceVector.block(3*ii, 0, 3, 1) = totalExternalForce;
		}
	}

	std::vector<Eigen::Vector3f> elemDeformedCoords;

	for (int ii = 0; ii < elem->vertices.size(); ++ii)
	{
		elemDeformedCoords.push_back(mesh->coords[elem->vertices[ii]]);
	}
		
	for (int ii = 0; ii < 8; ++ii)
	{
		// again, fix base of single element
		if (elem->vertices[ii] < 4)
		{
			continue;
		}

		Eigen::Vector3f forceOnVertex = elem->getForce(elemDeformedCoords, ii);

		totalForceVector.block(3*ii, 0, 3, 1) = totalForceVector.block(3*ii, 0, 3, 1) + forceOnVertex;
	}			


	Eigen::MatrixXf K = elem->stiffnessMatrix(elemDeformedCoords);
	Eigen::VectorXf deltaX = K.colPivHouseholderQr().solve(totalForceVector);


	std::cout << "totalForceVector: " << totalForceVector << std::endl;
	std::cout << "K:" << K << std::endl;
	std::cout << "deltaX:" << deltaX << std::endl;

	for (int ii = 0; ii < 8; ++ii)
	{
		int sharedCoordIndex = elem->vertices[ii];

		if (sharedCoordIndex < 4)
		{
			continue; 
		}

		mesh->coords[sharedCoordIndex] += stepSize * deltaX.block(3*ii, 0, 3, 1);
	}
}