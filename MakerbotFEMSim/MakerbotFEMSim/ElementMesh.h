#pragma once

#include "Element.h"
#include <vector>
#include <Eigen/Core>

class ElementMesh
{
public:
	ElementMesh(void);
	
	std::vector<Eigen::Vector3f> coords;

	std::vector<Element> elements;

	std::vector<Eigen::Vector3f> externalForces;

};

