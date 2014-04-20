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

//ElementMesh* MeshBuilder::buildGenericCubeMesh(float numX, float numY, float numZ)
//{
// 
//	std::vector<Eigen::Vector3f> zCoords;
//    int verticesPerElement = 8;
//
//
//    float size = 1.0f;
//
//	Eigen::Vector3f corner = Eigen::Vector3f::Zero();
//
//	std::vector<int> vertices;
//
//
//	std::vector<Element*> elements;
//
//    zCoords.push_back(corner); // 0
//	zCoords.push_back(corner + Eigen::Vector3f(0,0,size)); // 1
//	zCoords.push_back(corner + Eigen::Vector3f(0,size,0)); // 2
//	zCoords.push_back(corner + Eigen::Vector3f(0,size,size)); // 3
//	zCoords.push_back(corner + Eigen::Vector3f(size,0,0)); // 4
//	zCoords.push_back(corner + Eigen::Vector3f(size,0,size)); // 5
//	zCoords.push_back(corner + Eigen::Vector3f(size,size,0)); // 6
//	zCoords.push_back(corner + Eigen::Vector3f(size,size,size)); // 7 
//
//	std::vector<int> vertices;
//	for (int ii = 0; ii < 8; ++ii)
//	{
//		vertices.push_back(ii);
//	}
//	Element * onlyElem = new HexElement(vertices);
//	elements.push_back(onlyElem);
//
//
//
//    int nextCoordIndex = 8;
//    int totalCoordInCube = 8 + 4 * (numX-1);
//    int totalCoordInCubeRow = totalCoordInCube * (numY);
//    for(int kk = 0; kk < numZ; ++kk)
//    {
//
//
//        for (int jj = 0; jj < numY; ++jj)
//        {
//
//            corner = Eigen::Vector3f(0, size*(jj), size*kk);
//            nextCoordIndex = 8;
//            vertices.clear();
//            if (jj != 0)
//            {
//	            //zCoords.push_back(corner); // 0
//	            //zCoords.push_back(corner + Eigen::Vector3f(0,0,size)); // 1
//	            zCoords.push_back(corner + Eigen::Vector3f(0,size,0)); // 2
//	            zCoords.push_back(corner + Eigen::Vector3f(0,size,size)); // 3
//	            //zCoords.push_back(corner + Eigen::Vector3f(size,0,0)); // 4
//	           // zCoords.push_back(corner + Eigen::Vector3f(size,0,size)); // 5
//	            zCoords.push_back(corner + Eigen::Vector3f(size,size,0)); // 6
//	            zCoords.push_back(corner + Eigen::Vector3f(size,size,size)); // 7 
//                for (int i = totalCoordInCubeRow*kk + jj*totalCoordInCube; i < 8 + totalCoordInCubeRow*kk + jj*totalCoordInCube; ++i)
//	            {
//		            vertices.push_back(i);
//	            }
//
//	            Element * firstElem = new HexElement(vertices);
//	            elements.push_back(firstElem);
//            }
//
//            for (int ii = 0; ii < numX-1; ++ii)
//            {
//         
//                vertices.clear();
//                corner += Eigen::Vector3f(size, 0, 0);
//	            zCoords.push_back(corner + Eigen::Vector3f(size,0,0)); // 4
//	            zCoords.push_back(corner + Eigen::Vector3f(size,0,size)); // 5
//	            zCoords.push_back(corner + Eigen::Vector3f(size,size,0)); // 6
//	            zCoords.push_back(corner + Eigen::Vector3f(size,size,size)); // 7
//
//
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex-4);
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex-3);
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex-2);
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex-1);
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex);
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex+1);
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex+2);
//                vertices.push_back(totalCoordInCubeRow*kk + totalCoordInCube*jj + nextCoordIndex+3);
//
//                Element * nextElem = new HexElement(vertices);
//	            elements.push_back(nextElem); 
//                nextCoordIndex += 4;
//            }
//        }
//    }
//    ElementMesh * mesh = new ElementMesh();
//	mesh->coords = zCoords;
//	mesh->elements = elements;
//    
//	return mesh;
//
//}

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




