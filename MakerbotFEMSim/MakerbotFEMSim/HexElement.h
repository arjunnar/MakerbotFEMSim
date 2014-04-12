#pragma once

#include "Element.h"

class HexElement : Element
{
public:
	HexElement(void);
	Eigen::MatrixXf stiffnessMatrix(); // from deformation to stiffness
	
private: 
	Eigen::MatrixXf deformationGradient(int quadPoint); 


};

