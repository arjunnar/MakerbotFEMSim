#include "MeshBuilder.h"

ElementMesh* MeshBuilder::singleCubeMesh()
{
	std::vector<Eigen::Vector3f> zCoords;
	float size = 1.0f;

	Eigen::Vector3f corner = Eigen::Vector3f::Zero();
	zCoords.push_back(corner); // 0
	zCoords.push_back(corner + Eigen::Vector3f(0,0,size)); // 1
	zCoords.push_back(corner + Eigen::Vector3f(0,size,0)); // 2
	zCoords.push_back(corner + Eigen::Vector3f(0,size,size)); // 3
	zCoords.push_back(corner + Eigen::Vector3f(size,0,0)); // 4
	zCoords.push_back(corner + Eigen::Vector3f(size,0,size)); // 5
	zCoords.push_back(corner + Eigen::Vector3f(size,size,0)); // 6
	zCoords.push_back(corner + Eigen::Vector3f(size,size,size)); // 7 

	std::vector<int> vertices;
	for (int ii = 0; ii < 8; ++ii)
	{
		vertices.push_back(ii);
	}

	Element * onlyElem = new HexElement(vertices);
	std::vector<Element*> elements;
	elements.push_back(onlyElem);

	ElementMesh * mesh = new ElementMesh();
	mesh->coords = zCoords;
	mesh->elements = elements;

	return mesh;
}

std::vector<Eigen::Vector3f> MeshBuilder::getCubeVertices(Eigen::Vector3f corner, float size)
{
	std::vector<Eigen::Vector3f> cubeCoords;

	cubeCoords.push_back(corner); // 0
	cubeCoords.push_back(corner + Eigen::Vector3f(0,0,size)); // 1
	cubeCoords.push_back(corner + Eigen::Vector3f(0,size,0)); // 2
	cubeCoords.push_back(corner + Eigen::Vector3f(0,size,size)); // 3
	cubeCoords.push_back(corner + Eigen::Vector3f(size,0,0)); // 4
	cubeCoords.push_back(corner + Eigen::Vector3f(size,0,size)); // 5
	cubeCoords.push_back(corner + Eigen::Vector3f(size,size,0)); // 6
	cubeCoords.push_back(corner + Eigen::Vector3f(size,size,size)); // 7 

	return cubeCoords;
}




