#pragma once
#include "basestepper.h"
#include "HexElement.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

typedef Eigen::SparseMatrix<float, true> SparseMatrix;
typedef Eigen::Triplet<float> Triplet;

class NewtonMethodStepper : public BaseStepper
{

public:
	NewtonMethodStepper(ElementMesh * mesh);
	void step();

private:
	Eigen::Vector3f totalExternalForce;
};

