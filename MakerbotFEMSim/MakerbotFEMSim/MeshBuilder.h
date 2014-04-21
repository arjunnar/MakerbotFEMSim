#pragma once

#include "ElementMesh.h"
#include "Element.h"
#include "HexElement.h"
#include <Eigen/Core>
#include <iostream>
class MeshBuilder
{
public: 
	/*
	static ElementMesh* singleCubeMesh();
    static ElementMesh* twoStackedCubeMesh();
    */
	static ElementMesh* buildGenericCubeMesh(int numXDim, int numYDim, int numZDim, float size, std::vector<Eigen::Vector3f>  XX);

private:
	//static std::vector<Eigen::Vector3f> getCubeVertices(Eigen::Vector3f corner, float size);

};

