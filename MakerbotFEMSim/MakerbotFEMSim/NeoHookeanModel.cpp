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
	float J = log(defGrad.determinant());
	Eigen::Matrix3f invTranspose = defGrad.inverse().transpose();
	return mu * (defGrad - invTranspose) + lambda * J * invTranspose;
}