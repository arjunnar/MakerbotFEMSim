#pragma once

#include "Element.h"
#include "Quadrature.h"
#include "NeoHookeanModel.h"

//#define NVERT 8

class HexElement : public Element
{

public:
	HexElement(std::vector<int> vertices);
	Eigen::MatrixXf stiffnessMatrix(); // from deformation to stiffness?
	Eigen::Matrix3f defGradAtQuadPoint(Eigen::Vector3f p);
	Eigen::Vector3f getForce(int vertexIndex);
	Eigen::Vector3f getShapeFuncGrad(Eigen::Vector3f quadPoint, int ii);
private:
	Quadrature quadrature;
	Eigen::Vector3f refPoints[NVERT];
    int weights[8][3];
};

