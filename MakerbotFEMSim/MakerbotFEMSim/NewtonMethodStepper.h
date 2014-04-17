#pragma once

#include "ElementMesh.h"

class NewtonMethodStepper
{
public:
	NewtonMethodStepper(ElementMesh * mesh);
	void draw();
	void step();

private: 
	ElementMesh * mesh;
};

