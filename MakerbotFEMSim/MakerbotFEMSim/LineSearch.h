#pragma once

#include <Eigen/Core>
#include "ElementMesh.h"
#include "HexElement.h"

class LineSearch
{
public:
	static void advanceMesh(ElementMesh * mesh, Eigen::VectorXf &deltaX);

private:
	static void addDeltaX(ElementMesh * mesh, Eigen::VectorXf &deltaX, float stepSize);

};

