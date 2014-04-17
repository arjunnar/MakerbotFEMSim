#pragma once

#include "Element.h"

class HexElement : public Element
{

public:
	HexElement(std::vector<int> vertices);
	Eigen::MatrixXf stiffnessMatrix(); // from deformation to stiffness?
	
private: 
	Eigen::MatrixXf deformationGradient(int quadPoint); 


};

