#pragma once

#include "ElementMesh.h"
#include "Element.h"
#include "HexElement.h"
#include <Eigen/Core>
#include <iostream>
class MeshBuilder
{
public: 
	static ElementMesh* singleCubeMesh();
    static ElementMesh* twoStackedCubeMesh();
    //static ElementMesh* buildGenericCubeMesh(float numX, float numY, float numZ);

private:
	static std::vector<Eigen::Vector3f> getCubeVertices(Eigen::Vector3f corner, float size);

};

