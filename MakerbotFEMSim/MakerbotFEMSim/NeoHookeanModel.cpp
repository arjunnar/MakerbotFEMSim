#include "NeoHookeanModel.h"

const float NeoHookeanModel::mu = 1.056e5;
const float NeoHookeanModel::lambda = 7.038e4;

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

Eigen::Matrix3f NeoHookeanModel::dPdx(Eigen::Matrix3f F, Eigen::Matrix3f dF)
{
	float J = log(F.determinant());
	Eigen::Matrix3f Finv = F.inverse();
	Eigen::Matrix3f Finv_transpose = Finv.transpose();
	Eigen::Matrix3f dF_transpose = dF.transpose();

	Eigen::Matrix3f part1 = mu*dF;
	Eigen::Matrix3f part2 = (mu - lambda*J) * Finv_transpose * dF_transpose * Finv_transpose;
	Eigen::Matrix3f part3 = lambda * ((Finv*dF).trace()) * Finv_transpose;

	return part1 + part2 + part3;
}