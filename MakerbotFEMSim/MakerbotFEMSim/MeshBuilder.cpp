#include "MeshBuilder.h"

/*
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

ElementMesh* MeshBuilder::twoStackedCubeMesh()
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

	Element * firstElem = new HexElement(vertices);
	std::vector<Element*> elements;
	elements.push_back(firstElem);



    //stacked cube

	
    Eigen::Vector3f stackedCorner = Eigen::Vector3f(0, size, 0);

	zCoords.push_back(stackedCorner + Eigen::Vector3f(0,size,0)); // 8
	zCoords.push_back(stackedCorner + Eigen::Vector3f(0,size,size)); // 9
	zCoords.push_back(stackedCorner + Eigen::Vector3f(size,size,0)); // 10
	zCoords.push_back(stackedCorner + Eigen::Vector3f(size,size,size)); // 11

	std::vector<int> verticesStacked;
    verticesStacked.push_back(2);
    verticesStacked.push_back(3);
    verticesStacked.push_back(8);
    verticesStacked.push_back(9);
    verticesStacked.push_back(6);
    verticesStacked.push_back(7);
    verticesStacked.push_back(10);
    verticesStacked.push_back(11);

    
   	Element * secondElem = new HexElement(verticesStacked);
	elements.push_back(secondElem); 
    
    ElementMesh * mesh = new ElementMesh();
	mesh->coords = zCoords;
	mesh->elements = elements;
    
	return mesh;
}
*/

ElementMesh* MeshBuilder::buildGenericCubeMesh(int numXDim, int numYDim, int numZDim, float size, std::vector<Eigen::Vector3f> XX)
{
	int numLinesXDim = numXDim + 1;
	int numLinesYDim = numYDim + 1;
	int numLinesZDim = numZDim + 1;

	int sizeCoordBase = numLinesXDim * numLinesZDim;

	std::vector<Eigen::Vector3f> zCoords;
	for (int jj = 0; jj < numLinesYDim; ++jj) //adding one to get end of last cube
	{
		for (int kk = 0; kk < numLinesZDim; ++kk)
		{
			for (int ii = 0; ii < numLinesXDim; ++ii)
			{
				Eigen::Vector3f coord(ii*size, jj*size, kk*size);
				zCoords.push_back(coord);
			}
		}
	}

	std::vector<Element*> elements;
	

	for (int jj = 0; jj < numYDim; ++jj)
	{
		for (int kk = 0; kk < numZDim; ++kk)
		{
			for (int ii = 0; ii < numXDim; ++ii)
			{
				
				std::vector<int> vertices;
				vertices.push_back(ii + numLinesXDim * kk + sizeCoordBase * jj); // 0
				vertices.push_back(ii + numLinesXDim * (kk+1) + sizeCoordBase * jj); // 1
				vertices.push_back(ii + numLinesXDim * kk + sizeCoordBase * (jj+1)); // 2
				vertices.push_back(ii + numLinesXDim * (kk+1) + sizeCoordBase * (jj+1)); // 3
				vertices.push_back((ii + 1) + numLinesXDim * kk + sizeCoordBase * jj); // 4
				vertices.push_back((ii + 1) + numLinesXDim * (kk+1) + sizeCoordBase * jj); // 5
				vertices.push_back((ii + 1) + numLinesXDim * kk + sizeCoordBase * (jj + 1)); // 6
				vertices.push_back((ii + 1) + numLinesXDim * (kk + 1) + sizeCoordBase * (jj + 1)); // 7

				//std::cout << "vertex 0: " << vertices[0] << "\n";
				//std::cout << "vertex 1: " << vertices[1] << "\n";
				//std::cout << "vertex 2: " << vertices[2] << "\n";
				//std::cout << "vertex 3: " << vertices[3] << "\n";
				//std::cout << "vertex 4: " << vertices[4] << "\n";
				//std::cout << "vertex 5: " << vertices[5] << "\n";
				//std::cout << "vertex 6: " << vertices[6] << "\n";
				//std::cout << "vertex 7: " << vertices[7] << "\n";
				Element * elem = new HexElement(vertices, XX, Eigen::Vector3f::Zero());
				elements.push_back(elem);
			}
		}
	}
	std::set<int> sharedIndexBase;
	for (int ii = 0; ii < sizeCoordBase ; ++ii)
	{
		sharedIndexBase.insert(ii);
	}

	ElementMesh * mesh = new ElementMesh();
	mesh->coords = zCoords;
	mesh->elements = elements;
    mesh->sharedIndexBase = sharedIndexBase;

	return mesh;
}

/*

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
*/




