#pragma once

#include <vector>
#include <Eigen/Core>

class Element
{
public:
	Element();

	std::vector<int> vertices; // indexes into global list of coordinates 

	virtual Eigen::MatrixXf stiffnessMatrix() = 0;
};

