#include "NewtonMethodStepper.h"

NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh, bool useSparse) : BaseStepper(mesh)
{
	totalExternalForce = Eigen::Vector3f::Zero();
	Eigen::Vector3f force(-0.02, -0.05,0);
	mesh->externalForcesPerVertex.push_back(force);
	for (int i = 0; i < mesh->externalForcesPerVertex.size(); ++i)
	{
		totalExternalForce += mesh->externalForcesPerVertex[i];
	}

	this->useSparse = useSparse;
}

Eigen::MatrixXf NewtonMethodStepper::denseStiffMat(ElementMesh * mesh)
{
	int numNonFixedVertices = mesh->getNumNonFixedVertices();

	Eigen::MatrixXf K(3*mesh->coords.size(), 3*mesh->coords.size());
	K.setZero();

	for (int elementI = 0; elementI < mesh->elements.size(); ++elementI)
	{
		HexElement * elem = (HexElement*) mesh->elements[elementI];
		std::vector<Eigen::Vector3f> elemDeformedCoords; 

		for (int ii = 0; ii < elem->vertices.size(); ++ii)
		{
			elemDeformedCoords.push_back(mesh->coords[elem->vertices[ii]]);
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
				
				// DENSE MATRIX CODE
				K.block(3*rowSharedCoordIndex, 3*colSharedCoordIndex, 3, 3) += elementKBlock;
			}
		}
	}

	Eigen::MatrixXf newK(3*numNonFixedVertices, 3*numNonFixedVertices);
	newK.setZero();

	int nRowsNonFixed = 0;
	for (int rowI = 0; rowI < mesh->coords.size(); ++rowI)
	{
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

	return newK;
}

SparseMatrix NewtonMethodStepper::sparseStiffMat(ElementMesh * mesh)
{
	int numNonFixedVertices = mesh->getNumNonFixedVertices();
	std::vector<Triplet> tripletsK;

	for (int elementI = 0; elementI < mesh->elements.size(); ++elementI)
	{
		HexElement * elem = (HexElement*) mesh->elements[elementI];
		std::vector<Eigen::Vector3f> elemDeformedCoords; 

		for (int ii = 0; ii < elem->vertices.size(); ++ii)
		{
			elemDeformedCoords.push_back(mesh->coords[elem->vertices[ii]]);
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

				for (int r = 0; r < 3; ++r)
				{
					for (int c = 0; c < 3; ++c)
					{
						if (elementKBlock(r,c) != 0.0f)
						{
							tripletsK.push_back( Triplet(3*rowSharedCoordIndex + r, 3*colSharedCoordIndex + c, elementKBlock(r,c)) );
						}
					}
				}
			}
		}
	}

	SparseMatrix K(3*mesh->coords.size(), 3*mesh->coords.size());
	K.setFromTriplets(tripletsK.begin(), tripletsK.end());
	tripletsK.clear();
	std::vector<Triplet> tripletsNewK;

	int nRowsNonFixed = 0;
	for (int rowI = 0; rowI < mesh->coords.size(); ++rowI)
	{
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

			for (int r = 0; r < 3; ++r)
			{
				for (int c = 0; c < 3; ++c)
				{
					tripletsNewK.push_back( Triplet(3*nRowsNonFixed + r, 3*nColsNonFixed + c, K.coeff(3*rowI + r, 3*colI + c)) );
				}
			}
			
			++nColsNonFixed;
		}
		
		++nRowsNonFixed;
	}

	SparseMatrix newK(3*numNonFixedVertices, 3*numNonFixedVertices);;
	newK.setFromTriplets(tripletsNewK.begin(), tripletsNewK.end());
	tripletsNewK.clear();

	return newK;
}

void NewtonMethodStepper::step()
{
	std::cout << "Taking Newton's Method step" << std::endl;
	float stepSize = 0.01f; // TODO: Needs to be adaptively determined

	int numNonFixedVertices = 0;
	std::vector<int> nonFixedIndexes; 
	Eigen::VectorXf totalForceVector(3*mesh->coords.size());
	totalForceVector.setZero();

	// HANDLE EXTERNAL FORCES
	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		if (mesh->sharedIndexBase.count(sharedCoordI) == 0)
		{
			++numNonFixedVertices;
			nonFixedIndexes.push_back(sharedCoordI);

			if (sharedCoordI >= 72)//11*11*39 )//25*16)
			{
				totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce;  // TODO
			}
		}

		else 
		{
			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = Eigen::Vector3f::Zero();
		}
	}

	// COMPUTE FORCE ON EACH SHARED VERTEX
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
	}

	// REMOVE FIXED VERTICES FROM TOTAL FORCE VECTOR
	Eigen::VectorXf newForce(3*numNonFixedVertices);
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
	
	// SOLVE FOR DELTA X USING SPARSE OR DENSE SOLVER
	Eigen::VectorXf deltaX(3*numNonFixedVertices);
	if (this->useSparse)
	{
		SparseMatrix K = NewtonMethodStepper::sparseStiffMat(this->mesh);
		Eigen::ConjugateGradient<SparseMatrix> cg;
		cg.compute(K);
		deltaX = cg.solve(newForce);
	}

	else 
	{
		Eigen::MatrixXf K = NewtonMethodStepper::denseStiffMat(this->mesh);
		deltaX = K.colPivHouseholderQr().solve(newForce);
	}

	// UPDATE MESH COORDS
	for (int ii = 0; ii < numNonFixedVertices; ++ii)
	{
		int sharedCoordIndex = nonFixedIndexes[ii];
		mesh->coords[sharedCoordIndex] += stepSize * deltaX.block(3*ii, 0, 3, 1);
	}
}