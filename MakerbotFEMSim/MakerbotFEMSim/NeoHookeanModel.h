#pragma once

#include <Eigen/Core>
class NeoHookeanModel
{
public:
	NeoHookeanModel(float shearModulus, float lame);

    float getStrainEnergy(Eigen::Matrix3f defGrad);
	Eigen::Matrix3f firstPiolaStress(Eigen::Matrix3f defGrad);
private:
	float mu;
	float lambda;


};

