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
	NewtonMethodStepper(ElementMesh * mesh, bool useSparse);
	void step();
	static SparseMatrix sparseStiffMat(ElementMesh * mesh);
	static Eigen::MatrixXf denseStiffMat(ElementMesh * mesh);

private:
	Eigen::Vector3f totalExternalForce;
	bool useSparse;
};

