#pragma once

#include "ElementMesh.h"
#include "Element.h"
#include "HexElement.h"
#include <Eigen/Core>

class MeshBuilder
{
public: 
	static ElementMesh* singleCubeMesh();
    static ElementMesh* twoStackedCubeMesh();
private:
	static std::vector<Eigen::Vector3f> getCubeVertices(Eigen::Vector3f corner, float size);

};

