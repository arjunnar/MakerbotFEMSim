#pragma once

#include <Eigen/Core>
#include <vector>

#define NVERT 8

class Quadrature
{
public:
    Quadrature(void);
    ~Quadrature(void);

    Eigen::Vector3f gaussCubePoints[NVERT];
    float weights[NVERT];

};

