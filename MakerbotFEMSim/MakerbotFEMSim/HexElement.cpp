#include "HexElement.h"


HexElement::HexElement(std::vector<int> vertices) : Element(vertices)
{

}

Eigen::MatrixXf HexElement::stiffnessMatrix()
{
	//return NULL;
	return Eigen::Matrix3f();
}

