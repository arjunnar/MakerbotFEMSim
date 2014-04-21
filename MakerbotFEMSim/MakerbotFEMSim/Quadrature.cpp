#include "Quadrature.h"


Quadrature::Quadrature(void)
{
    float gauss = sqrt(3)/3;
    gaussCubePoints[0] = gauss*Eigen::Vector3f(-1,-1,-1);
    gaussCubePoints[1] = gauss*Eigen::Vector3f(-1,-1,1);
    gaussCubePoints[2] = gauss*Eigen::Vector3f(-1,1,-1);
    gaussCubePoints[3] = gauss*Eigen::Vector3f(-1,1,1);
    gaussCubePoints[4] = gauss*Eigen::Vector3f(1,-1,-1);
    gaussCubePoints[5] = gauss*Eigen::Vector3f(-1,1,-1);
    gaussCubePoints[6] = gauss*Eigen::Vector3f(1,1,-1);
    gaussCubePoints[7] = gauss*Eigen::Vector3f(1,1,1);

    for (int ii = 0; ii < 8; ++ii)
    {
        weights[ii] = 1.0/8.0;       
    }

}


Quadrature::~Quadrature(void)
{
}
