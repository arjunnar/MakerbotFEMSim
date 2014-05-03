#pragma once

#include "ElementMesh.h"
#include "HexElement.h"
#include "BaseStepper.h"
#include <iostream>

class GradientDescentStepper : public BaseStepper
{
public:
	GradientDescentStepper(ElementMesh * mesh);
	void draw();
	void step();

private: 
	Eigen::Vector3f totalExternalForce;
};

