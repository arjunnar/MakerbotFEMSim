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
	float stepSize = 1.0f;


	int numNonFixedVertices = 0;
	std::vector<int> nonFixedIndexes; 
	Eigen::VectorXf totalForceVector(3*mesh->coords.size());
	totalForceVector.setZero();
	Eigen::MatrixXf K(3*mesh->coords.size(), 3*mesh->coords.size());
	K.setZero();

	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		if (mesh->sharedIndexBase.count(sharedCoordI) == 0)
		{
			++numNonFixedVertices;
			nonFixedIndexes.push_back(sharedCoordI);
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce;  // TODO
		}

		else 
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = Eigen::Vector3f::Zero();
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
			
			if (mesh->sharedIndexBase.count(sharedCoordIndex) > 0)
			{
				continue;
			}

			Eigen::Vector3f forceOnVertex = elem->getForce(elemDeformedCoords, ii);

			totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) = totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) + forceOnVertex;
		}
		

		// put element K into total K
		Eigen::MatrixXf elementK = elem->stiffnessMatrix(elemDeformedCoords);
		for (int rowI = 0; rowI < elem->vertices.size(); ++rowI) // rows
		{
			int rowSharedCoordIndex = elem->vertices[rowI];

			for (int colI = 0; colI < elem->vertices.size(); ++colI) // columns
			{
				int colSharedCoordIndex = elem->vertices[colI];
				Eigen::Matrix3f elementKBlock = elementK.block(3*rowI, 3*colI, 3, 3);
				K.block(3*rowSharedCoordIndex, 3*colSharedCoordIndex, 3, 3) += elementKBlock;
			}
		}
	}

	Eigen::MatrixXf newK(3*numNonFixedVertices, 3*numNonFixedVertices);
	newK.setZero();

	Eigen::VectorXf newForce(3*numNonFixedVertices);
	newForce.setZero();

	int nRowsNonFixed = 0;
	for (int rowI = 0; rowI < mesh->coords.size(); ++rowI)
	{
		// row fixed
		if (mesh->sharedIndexBase.count(rowI) > 0)
		{
			continue;
		}

		
		int nColsNonFixed = 0;
		for (int colI = 0; colI < mesh->coords.size(); ++colI)
		{
			if (mesh->sharedIndexBase.count(colI) > 0)
			{
				continue;
			}

			newK.block(3*nRowsNonFixed, 3*nColsNonFixed, 3, 3) += K.block(3*rowI, 3*colI, 3, 3);
			++nColsNonFixed;
		}
		
		++nRowsNonFixed;
	}
	
	int nonFixedCount = 0;
	for (int ii = 0; ii < mesh->coords.size(); ++ii)
	{
		if (mesh->sharedIndexBase.count(ii) > 0)
		{
			continue; 
		}

		newForce.block(3*nonFixedCount, 0, 3, 1) += totalForceVector.block(3*ii, 0, 3, 1);
		++nonFixedCount;
	}

	Eigen::VectorXf deltaX = newK.colPivHouseholderQr().solve(newForce);

	//std::cout << "newK: " << newK << std::endl;
	//std::cout << "newForce: " << newForce << std::endl;
	//std::cout << "deltaX: " << deltaX << std::endl;

	for (int ii = 0; ii < numNonFixedVertices; ++ii)
	{
		int sharedCoordIndex = nonFixedIndexes[ii];

		mesh->coords[sharedCoordIndex] += stepSize * deltaX.block(3*ii, 0, 3, 1);
	}
}