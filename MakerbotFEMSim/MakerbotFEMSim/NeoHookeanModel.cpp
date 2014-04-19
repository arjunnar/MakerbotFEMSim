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

Eigen::Matrix3f  NeoHookeanModel::firstPiolaStress(Eigen::Matrix3f defGrad)
{
	float J    = log(defGrad.determinant());
	//Eigen::Matrix<float, 3,3> transposed = defGrad.transpose();
	Eigen::Matrix<float, 3,3> invTranspose = defGrad.inverse();
	return mu * (defGrad - invTranspose) + lambda * J * invTranspose;

}


