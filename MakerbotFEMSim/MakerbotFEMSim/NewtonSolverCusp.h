//#include "ElementMesh.h"
#include <vector>
class NewtonSolverCusp
{

public:
	static std::vector<float> step(std::vector<int> &I, std::vector<int> &J, std::vector<float> &V, std::vector<float> &force);
};
