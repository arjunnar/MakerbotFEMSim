#pragma once

#include "ElementMesh.h"

class BaseStepper
{
public:
	BaseStepper(ElementMesh * mesh);
	virtual void step() = 0;
protected:
	ElementMesh * mesh;
};

