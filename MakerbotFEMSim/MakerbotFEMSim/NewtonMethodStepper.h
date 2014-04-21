#pragma once

#include "ElementMesh.h"
#include "HexElement.h"

class NewtonMethodStepper
{
public:
	NewtonMethodStepper(ElementMesh * mesh);
	void draw();
	void step();

private: 
	ElementMesh * mesh;
	Eigen::Vector3f totalExternalForce;
};

