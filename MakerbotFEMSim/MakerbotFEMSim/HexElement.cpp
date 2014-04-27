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

	 for (int i = 0; i < NVERT; ++i)
	 {
		 std::cout << refPoints[i] << std::endl;
	 }
}

Eigen::MatrixXf HexElement::stiffnessMatrix()
{
	//return NULL;
	return Eigen::MatrixXf();
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
