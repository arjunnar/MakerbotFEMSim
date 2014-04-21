#include "NeoHookeanModel.h"

const float NeoHookeanModel::mu = 50.0f;
const float NeoHookeanModel::lambda = 50.0f;

float NeoHookeanModel::getStrainEnergy(Eigen::Matrix3f defGrad)
{
	float IOne = (defGrad.transpose() * defGrad).trace();
	float J    = log(defGrad.determinant());
	float strainEnergy  = mu / 2 * (IOne - 3) - mu * J + lambda / 2 * J * J;
	return strainEnergy;
}

Eigen::Matrix3f  NeoHookeanModel::firstPiolaStress(const Eigen::Matrix3f defGrad)
{
	//std::cout << "det: " << defGrad.determinant() << std::endl;
	float J = log(defGrad.determinant());
	Eigen::Matrix3f invTranspose = defGrad.inverse().transpose();
	//std::cout << "def grad inv transpose: " << invTranspose << std::endl;
	Eigen::Matrix3f PK1 = mu * (defGrad - invTranspose) + lambda * J * invTranspose;
	//std::cout << "PK1 in call: " << PK1 << std::endl;
	return PK1;
}