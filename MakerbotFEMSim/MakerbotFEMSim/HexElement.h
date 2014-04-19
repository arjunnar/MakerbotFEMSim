#pragma once

#include "Element.h"
#include "Quadrature.h"

//#define NVERT 8

class HexElement : public Element
{

public:
	HexElement(std::vector<int> vertices);
	Eigen::MatrixXf stiffnessMatrix(); // from deformation to stiffness?
	Eigen::Matrix3f defGradAtQuadPoint(Eigen::Vector3f p);

private:
	Quadrature quadrature;
	Eigen::Vector3f refPoints[NVERT];
    int weights[8][3];
	
};

