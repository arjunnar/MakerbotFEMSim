#include "NewtonMethodStepper.h"

NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh, bool useSparse) : BaseStepper(mesh)
{
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
		if (mesh->fixedVertexIndexes.count(rowI) > 0)
		{
			continue;
		}

		int nColsNonFixed = 0;
		for (int colI = 0; colI < mesh->coords.size(); ++colI)
		{
			if (mesh->fixedVertexIndexes.count(colI) > 0)
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

std::vector<Triplet> NewtonMethodStepper::sparseStiffMat(ElementMesh * mesh)
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
						if (abs(elementKBlock(r,c)) > 1.0e-10)
						{
							int I = mesh->originalToNewIndexes[3*rowSharedCoordIndex + r];
							int J = mesh->originalToNewIndexes[3*colSharedCoordIndex + c];

							if (I == -1 || J == -1)
							{
								continue;
							}

							float val = elementKBlock(r,c);
							tripletsK.push_back(Triplet(I,J,val));
						}
					}
				}
			}
		}
	}

	/*
	int nRowsNonFixed = 0;
	for (int rowI = 0; rowI < mesh->coords.size(); ++rowI)
	{
		if (mesh->fixedVertexIndexes.count(rowI) > 0)
		{
			continue;
		}

		int nColsNonFixed = 0;
		for (int colI = 0; colI < mesh->coords.size(); ++colI)
		{
			if (mesh->fixedVertexIndexes.count(colI) > 0)
			{
				continue;
			}

			for (int r = 0; r < 3; ++r)
			{
				for (int c = 0; c < 3; ++c)
				{
					if (abs(K.coeff(3*rowI + r, 3*colI + c)) < 1e-10)
						continue;
					tripletsNewK.push_back( Triplet(3*nRowsNonFixed + r, 3*nColsNonFixed + c, K.coeff(3*rowI + r, 3*colI + c)) );
				}
			}
			
			++nColsNonFixed;
		}
				
		++nRowsNonFixed;
	}
	*/

	return tripletsK;
}

Eigen::VectorXf NewtonMethodStepper::getTotalForceVector(ElementMesh * mesh)
{
	Quadrature quadrature;
    Eigen::Vector3f totalExternalForce(-0.02, -0.05,0);
	int numNonFixedVertices = mesh->getNumNonFixedVertices();
	Eigen::VectorXf totalForceVector(3*mesh->coords.size());
	totalForceVector.setZero();

	// HANDLE EXTERNAL FORCES
	for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	{
		if (mesh->fixedVertexIndexes.count(sharedCoordI) == 0)
		{
			if (sharedCoordI >= 72)//21*21*46)//9*9*31)//11*11*39 )//25*16)
			{
				totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce; 
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
			
			if (mesh->fixedVertexIndexes.count(sharedCoordIndex) > 0)
			{
				continue;
			}

			Eigen::Vector3f forceOnVertex = elem->getForce(elemDeformedCoords, ii);

			totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) = totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) + forceOnVertex;
		}
	}

	// REMOVE FIXED VERTICES FROM TOTAL FORCE VECTOR
	Eigen::VectorXf newForce(3*numNonFixedVertices);
    newForce.setZero();
	int nonFixedCount = 0;
	for (int ii = 0; ii < mesh->coords.size(); ++ii)
	{
		if (mesh->fixedVertexIndexes.count(ii) > 0)
		{
			continue; 
		}

		newForce.block(3*nonFixedCount, 0, 3, 1) += totalForceVector.block(3*ii, 0, 3, 1);
		++nonFixedCount;
	}

	return newForce;
}

void NewtonMethodStepper::step()
{
	std::cout << "Taking Newton's Method step" << std::endl;
	Eigen::VectorXf force = NewtonMethodStepper::getTotalForceVector(this->mesh);
	Eigen::VectorXf deltaX(3*mesh->getNumNonFixedVertices());
	
	if (this->useSparse)
	{
		std::vector<Triplet> tripletList = NewtonMethodStepper::sparseStiffMat(this->mesh);
		SparseMatrix K(3*mesh->getNumNonFixedVertices(), 3*mesh->getNumNonFixedVertices());
		K.setFromTriplets(tripletList.begin(), tripletList.end());
		tripletList.clear();

		Eigen::ConjugateGradient<SparseMatrix> cg;
		cg.compute(K);
		deltaX = cg.solve(force);
	}

	else 
	{
		Eigen::MatrixXf K = NewtonMethodStepper::denseStiffMat(this->mesh);
		deltaX = K.colPivHouseholderQr().solve(force);
	}

	// UPDATE MESH COORDS
	LineSearch::advanceMesh(mesh, deltaX);
}