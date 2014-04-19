#include "HexElement.h"


HexElement::HexElement(std::vector<int> vertices) : Element(vertices)
{
	// initialize weights matrix
	 weights[0][0] = -1; weights[0][1] = -1; weights[0][2] = -1; 
	 weights[1][0] = -1; weights[1][1] = -1; weights[1][2] = 1; 
	 weights[2][0] = -1; weights[2][1] = 1; weights[1][2] = 11; 
	 weights[3][0] = -1; weights[3][1] = 1; weights[3][2] = 1; 			
	 weights[4][0] = 1; weights[4][1] = -1; weights[4][2] = -1; 
	 weights[5][0] = 1; weights[5][1] = -1; weights[5][2] = 1; 
	 weights[6][0] = 1; weights[6][1] = 1; weights[6][2] = -1;  
	 weights[7][0] = 1; weights[7][1] = 1; weights[7][2] = 1;  

	// initialize cube in reference space 
}

Eigen::MatrixXf HexElement::stiffnessMatrix()
{
	//return NULL;
	return Eigen::Matrix3f();
}

// function to find F_j
Eigen::Matrix3f HexElement::defGradAtQuadPoint(Eigen::Vector3f quadPoint)
{
	Eigen::Matrix3f defGradQuad = Eigen::Matrix3f::Identity();
	Eigen::Vector3f fourTimesDiag = 4 * (refPoints[7] - refPoints[0]);

	for (int ii = 0; ii < NVERT; ++ii)
	{
		Eigen::Vector3f shapeFunctionGrad;
		shapeFunctionGrad(0) = (1+weights[ii][1]*quadPoint(1)) * (1+weights[ii][2]*quadPoint(2)) / fourTimesDiag(0);
		shapeFunctionGrad(1) = (1+weights[ii][0]*quadPoint(0)) * (1+weights[ii][2]*quadPoint(2)) / fourTimesDiag(1);
		shapeFunctionGrad(2) = (1+weights[ii][0]*quadPoint(0)) * (1+weights[ii][1]*quadPoint(1)) / fourTimesDiag(2);

		Eigen::MatrixXf defGradContribution = quadPoint * shapeFunctionGrad.transpose();
		defGradQuad += defGradContribution;
	}

	return defGradQuad;
}
