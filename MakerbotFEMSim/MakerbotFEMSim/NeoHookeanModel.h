#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>

class NeoHookeanModel
{
public:
	//NeoHookeanModel(float shearModulus, float lame);
    static float getStrainEnergy(Eigen::Matrix3f defGrad);
	static Eigen::Matrix3f firstPiolaStress(Eigen::Matrix3f defGrad);
	static Eigen::Matrix3f dPdx(Eigen::Matrix3f defGrad, Eigen::Matrix3f dF);
private:
	static const float mu; 
	static const float lambda; 
};

