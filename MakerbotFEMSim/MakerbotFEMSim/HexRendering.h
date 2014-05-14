#pragma once

#include "extra.h"
#include "ElementMesh.h"

class HexRendering
{
public:
	static void drawHexMesh(ElementMesh * mesh);
	static void drawHexMeshWithStress(ElementMesh * mesh, std::vector<Eigen::Matrix3f> &cauchyStressTensors);

private:
	static void drawLine(Eigen::Vector3f x0, Eigen::Vector3f x1);

};


