#pragma once
#include "basestepper.h"
#include "HexElement.h"

#include <Eigen/Dense>

class NewtonMethodStepper :
	public BaseStepper
{
public:
	NewtonMethodStepper(ElementMesh * mesh);
	void step();
private:
	Eigen::Vector3f totalExternalForce;

};

