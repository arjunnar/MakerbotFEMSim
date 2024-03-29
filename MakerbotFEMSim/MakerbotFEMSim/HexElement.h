#pragma once

#include "Element.h"
#include "Quadrature.h"
#include "NeoHookeanModel.h"
#include <iostream>
#include <vector>

//#define NVERT 8

#define KDIM NVERT*3

class HexElement : public Element
{

public:
	HexElement(std::vector<int> vertices, std::vector<Eigen::Vector3f> XX, Eigen::Vector3f corner);
	Eigen::MatrixXf stiffnessMatrix(std::vector<Eigen::Vector3f> deformedCoords); // from deformation to stiffness?
	Eigen::Matrix3f defGradAtQuadPoint(std::vector<Eigen::Vector3f> deformedCoords, Eigen::Vector3f p);
	Eigen::Vector3f getForce(std::vector<Eigen::Vector3f> deformedCoords, int vertexIndex);
	Eigen::Vector3f getShapeFuncGrad(Eigen::Vector3f quadPoint, int ii);
private:
	Quadrature quadrature;
	std::vector<Eigen::Vector3f> refPoints;
    int weights[8][3];

	// helper functions
	Eigen::MatrixXf KAtQuadPoint(float weight, Eigen::Vector3f quadPoint, Eigen::Matrix3f Fj);
};

