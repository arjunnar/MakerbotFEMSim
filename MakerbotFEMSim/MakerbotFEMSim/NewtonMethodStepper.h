#pragma once
#include "basestepper.h"
#include "HexElement.h"
#include "LineSearch.h"

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Sparse>


typedef Eigen::SparseMatrix<float, true> SparseMatrix;
typedef Eigen::Triplet<float> Triplet;

class NewtonMethodStepper : public BaseStepper
{

public:
	NewtonMethodStepper(ElementMesh * mesh, bool useSparse);
	void step();
	
	static std::vector<Triplet> sparseStiffMat(ElementMesh * mesh);
	static Eigen::MatrixXf denseStiffMat(ElementMesh * mesh);
	static Eigen::VectorXf getTotalForceVector(ElementMesh * mesh);

private:
	bool useSparse;
};

