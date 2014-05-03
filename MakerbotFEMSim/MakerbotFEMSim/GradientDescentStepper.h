#pragma once

#include "ElementMesh.h"
#include "HexElement.h"
#include <iostream>

class GradientDescentStepper
{
public:
	GradientDescentStepper(ElementMesh * mesh);
	void draw();
	void step();

private: 
	ElementMesh * mesh;
	Eigen::Vector3f totalExternalForce;
};

