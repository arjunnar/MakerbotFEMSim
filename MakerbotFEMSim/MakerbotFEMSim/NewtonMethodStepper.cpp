#include "NewtonMethodStepper.h"


NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh) : BaseStepper(mesh)
{
	totalExternalForce = Eigen::Vector3f::Zero();
	Eigen::Vector3f force(1.0f,0,0);
	mesh->externalForcesPerVertex.push_back(force);
	for (int i = 0; i < mesh->externalForcesPerVertex.size(); ++i)
	{
		totalExternalForce += mesh->externalForcesPerVertex[i];
	}
}


void NewtonMethodStepper::step()
{
	std::cout << "Taking Newton's Method step" << std::endl;
	float stepSize = 0.02f;

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
		
		//std::cout << "force on vertex " << ii << ": " << forceOnVertex << std::endl;

		totalForceVector.block(3*ii, 0, 3, 1) = totalForceVector.block(3*ii, 0, 3, 1) + forceOnVertex;
	}			


	Eigen::MatrixXf K = elem->stiffnessMatrix(elemDeformedCoords);
	

	//std::cout << "totalForceVector: " << totalForceVector << std::endl;
	//std::cout << "K:" << K << std::endl;

	// remove fixed vertices (hard code this for now)
	Eigen::MatrixXf newK(12,12);
	newK.block(0,0,6,6) = K.block(6,6,6,6);
	newK.block(0,6,6,6) = K.block(6,18,6,6);
	newK.block(6,0,6,6) = K.block(18,6,6,6);
	newK.block(6,6,6,6) = K.block(18,18,6,6);
	
	Eigen::VectorXf newForce(12);
	newForce.block(0,0,6,1) = totalForceVector.block(6,0,6,1); // vertices 2 and 3
	newForce.block(6,0,6,1) = totalForceVector.block(18,0,6,1); // vertices 6 and 7	
	
	Eigen::VectorXf deltaX = newK.colPivHouseholderQr().solve(newForce);

	//std::cout << "newK: " << newK << std::endl;
	//std::cout << "newForce: " << newForce << std::endl;
	//std::cout << "deltaX: " << deltaX << std::endl;

	for (int ii = 0; ii < 8; ++ii)
	{
		int sharedCoordIndex = elem->vertices[ii];

		if (sharedCoordIndex < 4)
		{
			continue; 
		}

		int blockNum = -1;

		if (ii == 2)
			blockNum = 0;
		else if (ii == 3)
			blockNum = 1;
		else if (ii == 6)
			blockNum = 2;
		else if (ii == 7)
			blockNum = 3;

		mesh->coords[sharedCoordIndex] += stepSize * deltaX.block(3*blockNum, 0, 3, 1);
	}
}