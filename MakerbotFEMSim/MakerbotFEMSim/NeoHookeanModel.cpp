#include "NeoHookeanModel.h"


NeoHookeanModel::NeoHookeanModel(float shearModulus, float lame)
{
	mu = shearModulus;
	lambda = lame;
}


float NeoHookeanModel::getStrainEnergy(Eigen::Matrix3f defGrad)
{
	float IOne = (defGrad.transpose() * defGrad).trace();
	float J    = log(defGrad.determinant());

	float strainEnergy  = mu /2 * (IOne - 3) - mu*J + lambda / 2 * J * J;
	return strainEnergy;
}

Eigen::Matrix3f  NeoHookeanModel::firstPiolaStress(const Eigen::Matrix3f defGrad)
{
	float J = log(defGrad.determinant());
	Eigen::Matrix3f inverse = defGrad.inverse();
	return mu * (defGrad - inverse) + lambda * J * inverse;

}

