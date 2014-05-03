#pragma once

#include <vector>
#include <Eigen/Core>

class Element
{
public:
	Element(std::vector<int> vertices);

	virtual Eigen::MatrixXf stiffnessMatrix(std::vector<Eigen::Vector3f> deformedCoords) = 0;

	std::vector<int> vertices; // indexes into global list of coordinates
};

