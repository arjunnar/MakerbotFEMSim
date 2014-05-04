#include "HexElement.h"


HexElement::HexElement(std::vector<int> vertices, std::vector<Eigen::Vector3f> XX, Eigen::Vector3f corner) : Element(vertices)
{
	// initialize weights matrix
	 weights[0][0] = -1; weights[0][1] = -1; weights[0][2] = -1; 
	 weights[1][0] = -1; weights[1][1] = -1; weights[1][2] = 1; 
	 weights[2][0] = -1; weights[2][1] = 1; weights[2][2] = -1; 
	 weights[3][0] = -1; weights[3][1] = 1; weights[3][2] = 1; 			
	 weights[4][0] = 1; weights[4][1] = -1; weights[4][2] = -1; 
	 weights[5][0] = 1; weights[5][1] = -1; weights[5][2] = 1; 
	 weights[6][0] = 1; weights[6][1] = 1; weights[6][2] = -1;  
	 weights[7][0] = 1; weights[7][1] = 1; weights[7][2] = 1;  

	// initialize cube in reference space 
	 this->refPoints = XX;


	 for (int ii = 0; ii < NVERT; ++ii)
	 {
		 refPoints[ii] = refPoints[ii] + corner;
	 }
}

Eigen::MatrixXf HexElement::stiffnessMatrix(std::vector<Eigen::Vector3f> deformedCoords)
{
	Eigen::Vector3f diag = (refPoints[7] - refPoints[0]);
	float volume = diag[0]*diag[1]*diag[2];

	Eigen::MatrixXf K(KDIM, KDIM);
	K.setZero();
	
	for (int jj = 0; jj < NVERT; ++jj)
	{
		Eigen::Matrix3f Fj = defGradAtQuadPoint(deformedCoords, quadrature.gaussCubePoints[jj]);
		K += volume * KAtQuadPoint(quadrature.weights[jj], quadrature.gaussCubePoints[jj], Fj);
	}

	return K;
}

Eigen::MatrixXf HexElement::KAtQuadPoint(float weight, Eigen::Vector3f quadPoint, Eigen::Matrix3f Fj)
{
	Eigen::MatrixXf Kj(KDIM, KDIM);

	// calculate shape function gradient for each vertex at the specified quad point
	Eigen::Vector3f shapeFuncGrad[NVERT];
	for (int ii = 0; ii < NVERT; ++ii)
	{
		shapeFuncGrad[ii] = getShapeFuncGrad(quadPoint, ii);
	}

	// iterate through vertex positions 
	for (int ii = 0; ii < NVERT; ++ii)
	{
		// iterate through each direction of the vertex position
		// each direction give us a column in the matrix
		for (int col = 0; col < 3; ++col)
		{
			Eigen::Matrix3f dF;
			dF.setZero();
			dF.row(col) = shapeFuncGrad[ii].transpose();
			Eigen::Matrix3f dPdxInDirection = NeoHookeanModel::dPdx(Fj, dF);

			for (int forceNum = 0; forceNum < NVERT; ++forceNum)
			{
				Eigen::Vector3f dForceInDirection = dPdxInDirection*shapeFuncGrad[forceNum];
				Kj.block(forceNum*3, col, 3, 1) = weight*dForceInDirection;
			}
		}
	}

	return Kj;
}

// function to find F_j
Eigen::Matrix3f HexElement::defGradAtQuadPoint(std::vector<Eigen::Vector3f> deformedCoords, Eigen::Vector3f quadPoint)
{
	Eigen::Matrix3f defGradQuad = Eigen::Matrix3f::Identity();
	Eigen::Vector3f fourTimesDiag = 4 * (refPoints[7] - refPoints[0]);

	//std::cout << "refPoints 0 : " << refPoints[0] << std::endl;
	//std::cout << "refPoints 7 : " << refPoints[7] << std::endl;
	//std::cout << "four times diag: " << fourTimesDiag << std::endl;
	for (int ii = 0; ii < NVERT; ++ii)
	{
		Eigen::Vector3f shapeFunctionGrad;
		shapeFunctionGrad(0) = weights[ii][0] * (1+weights[ii][1]*quadPoint(1)) * (1+weights[ii][2]*quadPoint(2)) / fourTimesDiag(0);
		shapeFunctionGrad(1) = weights[ii][1] * (1+weights[ii][0]*quadPoint(0)) * (1+weights[ii][2]*quadPoint(2)) / fourTimesDiag(1);
		shapeFunctionGrad(2) = weights[ii][2] * (1+weights[ii][0]*quadPoint(0)) * (1+weights[ii][1]*quadPoint(1)) / fourTimesDiag(2);

		Eigen::MatrixXf defGradContribution = (deformedCoords[ii] - refPoints[ii]) * shapeFunctionGrad.transpose();
		defGradQuad += defGradContribution;
	}

	return defGradQuad;
}

Eigen::Vector3f HexElement::getForce(std::vector<Eigen::Vector3f> deformedCoords, int vertexIndex)
{
	Eigen::Vector3f force = Eigen::Vector3f(0,0,0);
	for (int jj = 0; jj < NVERT; ++jj)
	{
		Eigen::Vector3f quadPoint = quadrature.gaussCubePoints[jj];
		Eigen::Matrix3f defGrad = defGradAtQuadPoint(deformedCoords, quadPoint);
		//std::cout << "Def grad: " << defGrad << std::endl;
		Eigen::Vector3f shapeFuncGrad = getShapeFuncGrad(quadPoint, vertexIndex);
		//std::cout << "Shape func gradient: " << shapeFuncGrad << std::endl;
		Eigen::Matrix3f PK1 = NeoHookeanModel::firstPiolaStress(defGrad);
		//std::cout << "PK1: " << PK1 << std::endl;
		Eigen::Vector3f referenceDiagonal = (refPoints[7] - refPoints[0]);
		float refVolume = referenceDiagonal(0) * referenceDiagonal(1) * referenceDiagonal(2);
		force -= PK1 * shapeFuncGrad * refVolume * quadrature.weights[jj];
		//std::cout << "Pk1: " << std::endl << PK1 << std::endl;
	}

	return force;
}

Eigen::Vector3f HexElement::getShapeFuncGrad(Eigen::Vector3f quadPoint, int ii)
{
	Eigen::Vector3f shapeFuncGrad;
	Eigen::Vector3f fourTimesDiag = 4 * (refPoints[7] - refPoints[0]);
	shapeFuncGrad(0) = weights[ii][0] * (1+weights[ii][1]*quadPoint(1)) * (1+weights[ii][2]*quadPoint(2)) / fourTimesDiag(0);
	shapeFuncGrad(1) = weights[ii][1] * (1+weights[ii][0]*quadPoint(0)) * (1+weights[ii][2]*quadPoint(2)) / fourTimesDiag(1);
	shapeFuncGrad(2) = weights[ii][2] * (1+weights[ii][0]*quadPoint(0)) * (1+weights[ii][1]*quadPoint(1)) / fourTimesDiag(2);
	return shapeFuncGrad;
}
