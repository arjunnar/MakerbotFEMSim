#pragma once
#include "basestepper.h"
class NewtonMethodStepper :
	public BaseStepper
{
public:
	NewtonMethodStepper(ElementMesh * mesh);
	void step();
};

