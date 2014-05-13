#include "NewtonSolverCusp.h"

// general cuda includes
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

// cusp and thurst includes
#include <cusp/coo_matrix.h>
#include <cusp/print.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/inner_product.h>
#include <thrust/iterator/zip_iterator.h>
#include <cusp/krylov/cg.h>
#include <cusp/monitor.h>
#include <thrust/copy.h>


std::vector<float> NewtonSolverCusp::step(std::vector<int> &stdI, std::vector<int> &stdJ, std::vector<float> &stdV, std::vector<float> &stdForce)
{
	int num_triplets = stdI.size();

	cusp::array1d<int,   cusp::device_memory> I(num_triplets);  // row indices
    cusp::array1d<int,   cusp::device_memory> J(num_triplets);  // column indices
    cusp::array1d<float, cusp::device_memory> V(num_triplets);  // value
	
	thrust::copy(stdI.begin(), stdI.end(), I.begin());
	thrust::copy(stdJ.begin(), stdJ.end(), J.begin());
	thrust::copy(stdV.begin(), stdV.end(), V.begin());

	thrust::stable_sort_by_key(J.begin(), J.end(), thrust::make_zip_iterator(thrust::make_tuple(I.begin(), V.begin())));
    thrust::stable_sort_by_key(I.begin(), I.end(), thrust::make_zip_iterator(thrust::make_tuple(J.begin(), V.begin())));
	
	int num_entries = thrust::inner_product(thrust::make_zip_iterator(thrust::make_tuple(I.begin(), J.begin())),
                                            thrust::make_zip_iterator(thrust::make_tuple(I.end (),  J.end()))   - 1,
                                            thrust::make_zip_iterator(thrust::make_tuple(I.begin(), J.begin())) + 1,
                                            int(0),
                                            thrust::plus<int>(),
                                            thrust::not_equal_to< thrust::tuple<int,int> >()) + 1;

	int num_rows = stdForce.size();
	int num_cols = stdForce.size();

	cusp::coo_matrix<int, float, cusp::device_memory> K(num_rows, num_cols, num_entries);

    thrust::reduce_by_key(thrust::make_zip_iterator(thrust::make_tuple(I.begin(), J.begin())),
                          thrust::make_zip_iterator(thrust::make_tuple(I.end(),   J.end())),
                          V.begin(),
                          thrust::make_zip_iterator(thrust::make_tuple(K.row_indices.begin(), K.column_indices.begin())),
                          K.values.begin(),
                          thrust::equal_to< thrust::tuple<int,int> >(),
                          thrust::plus<float>());

	cusp::array1d<float, cusp::device_memory> x(K.num_rows, 0);
    cusp::array1d<float, cusp::device_memory> f(K.num_rows);

	thrust::copy(stdForce.begin(), stdForce.end(), f.begin());

	// set stopping criteria:
    //  iteration_limit    = 100
    //  relative_tolerance = 1e-3
    //cusp::verbose_monitor<float> monitor(f, 100, 1e-3);

	// set preconditioner (identity)
	cusp::identity_operator<float, cusp::device_memory> M(K.num_rows, K.num_rows);
	
	cusp::krylov::cg(K,x,f);

	std::vector<float> deltaX(K.num_rows);
	
	thrust::copy(x.begin(), x.end(), deltaX.begin());

	return deltaX;
}

//NewtonMethodStepper::NewtonMethodStepper(ElementMesh * mesh) : BaseStepper(mesh)
//{
	/*
	totalExternalForce = Eigen::Vector3f::Zero();
	Eigen::Vector3f force(0,-0.1,0);
	mesh->externalForcesPerVertex.push_back(force);
	for (int i = 0; i < mesh->externalForcesPerVertex.size(); ++i)
	{
		totalExternalForce += mesh->externalForcesPerVertex[i];
	}
	*/
//}


//void NewtonMethodStepper::step()
//{
	//std::cout << "Taking Newton's Method step" << std::endl;
	//
	//float stepSize = 0.01f; // TODO: Needs to be adaptively determined

	//int numNonFixedVertices = 0;
	//std::vector<int> nonFixedIndexes; 
	//Eigen::VectorXf totalForceVector(3*mesh->coords.size());
	//totalForceVector.setZero();

	//// DENSE MATRIX CODE
	////Eigen::MatrixXf K(3*mesh->coords.size(), 3*mesh->coords.size());
	////K.setZero();
	//
	//// SPARSE MATRIX CODE
	//std::vector<Triplet> tripletsK;

	//for (int sharedCoordI = 0; sharedCoordI < mesh->coords.size(); ++sharedCoordI)
	//{
	//	if (mesh->sharedIndexBase.count(sharedCoordI) == 0)
	//	{
	//		++numNonFixedVertices;
	//		nonFixedIndexes.push_back(sharedCoordI);

	//		if (sharedCoordI >= 72)//11*11*39 )//25*16)
	//		{
	//			totalForceVector.block(3*sharedCoordI, 0, 3, 1) = totalExternalForce;  // TODO
	//		}
	//	}

	//	else 
	//	{
	//		totalForceVector.block(3*sharedCoordI, 0, 3, 1) = Eigen::Vector3f::Zero();
	//	}
	//}

	//for (int elementI = 0; elementI < mesh->elements.size(); ++elementI)
	//{
	//	HexElement * elem = (HexElement*) mesh->elements[elementI];
	//	std::vector<Eigen::Vector3f> elemDeformedCoords; 

	//	for (int ii = 0; ii < elem->vertices.size(); ++ii)
	//	{
	//		elemDeformedCoords.push_back(mesh->coords[elem->vertices[ii]]);
	//	}
	//	
	//	for (int ii = 0; ii < elem->vertices.size(); ++ii)
	//	{
	//		int sharedCoordIndex = elem->vertices[ii];
	//		
	//		if (mesh->sharedIndexBase.count(sharedCoordIndex) > 0)
	//		{
	//			continue;
	//		}

	//		Eigen::Vector3f forceOnVertex = elem->getForce(elemDeformedCoords, ii);

	//		totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) = totalForceVector.block(3*sharedCoordIndex, 0, 3, 1) + forceOnVertex;
	//	}

	//	//std::cout << "Total Force Vector: " << totalForceVector << std::endl;

	//	// put element K into total K
	//	Eigen::MatrixXf elementK = elem->stiffnessMatrix(elemDeformedCoords);
	//	for (int rowI = 0; rowI < elem->vertices.size(); ++rowI) // rows
	//	{
	//		int rowSharedCoordIndex = elem->vertices[rowI];

	//		for (int colI = 0; colI < elem->vertices.size(); ++colI) // columns
	//		{
	//			int colSharedCoordIndex = elem->vertices[colI];
	//			Eigen::Matrix3f elementKBlock = elementK.block(3*rowI, 3*colI, 3, 3);
	//			
	//			// DENSE MATRIX CODE
	//			//K.block(3*rowSharedCoordIndex, 3*colSharedCoordIndex, 3, 3) += elementKBlock;

	//			// SPARSE MATRIX CODE
	//			for (int r = 0; r < 3; ++r)
	//			{
	//				for (int c = 0; c < 3; ++c)
	//				{
	//					if (elementKBlock(r,c) != 0.0f)
	//					{
	//						tripletsK.push_back( Triplet(3*rowSharedCoordIndex + r, 3*colSharedCoordIndex + c, elementKBlock(r,c)) );
	//					}
	//				}
	//			}
	//		}
	//	}
	//}

	//
	//// SPARSE MATRIC C
	//SparseMatrix K(3*mesh->coords.size(), 3*mesh->coords.size());
	//K.setFromTriplets(tripletsK.begin(), tripletsK.end());
	//tripletsK.clear();

	//Eigen::MatrixXf newK(3*numNonFixedVertices, 3*numNonFixedVertices);
	//newK.setZero();

	////cusp::array1d<int,   cusp::device_memory> I();  // row indices
 //   //cusp::array1d<int,   cusp::device_memory> J();  // column indices
 //   //cusp::array1d<float, cusp::device_memory> V();  // values

	//Eigen::VectorXf newForce = Eigen::VectorXf::Zero(3*numNonFixedVertices);

	//int nRowsNonFixed = 0;
	//for (int rowI = 0; rowI < mesh->coords.size(); ++rowI)
	//{
	//	// row fixed
	//	if (mesh->sharedIndexBase.count(rowI) > 0)
	//	{
	//		continue;
	//	}

	//	
	//	int nColsNonFixed = 0;
	//	for (int colI = 0; colI < mesh->coords.size(); ++colI)
	//	{
	//		if (mesh->sharedIndexBase.count(colI) > 0)
	//		{
	//			continue;
	//		}

	//		// DENSE MATRIX CODE
	//		// newK.block(3*nRowsNonFixed, 3*nColsNonFixed, 3, 3) += K.block(3*rowI, 3*colI, 3, 3);

	//		
	//		for (int r = 0; r < 3; ++r)
	//		{
	//			for (int c = 0; c < 3; ++c)
	//			{
	//				//I.
	//				//.push_back( Triplet(3*nRowsNonFixed + r, 3*nColsNonFixed + c, K.coeff(3*rowI + r, 3*colI + c)) );
	//			}
	//		}
	//		

	//		++nColsNonFixed;
	//	}
	//	
	//	++nRowsNonFixed;
	//}
	//
	///*
	//SparseMatrix newK(3*numNonFixedVertices, 3*numNonFixedVertices);;
	//newK.setFromTriplets(tripletsNewK.begin(), tripletsNewK.end());
	//tripletsNewK.clear();
	//*/

	//int nonFixedCount = 0;
	//for (int ii = 0; ii < mesh->coords.size(); ++ii)
	//{
	//	if (mesh->sharedIndexBase.count(ii) > 0)
	//	{
	//		continue; 
	//	}

	//	newForce.block(3*nonFixedCount, 0, 3, 1) += totalForceVector.block(3*ii, 0, 3, 1);
	//	++nonFixedCount;
	//}
	//
	////Eigen::ConjugateGradient<SparseMatrix> cg;
	////std::cout << "newK: " << newK;
	////cg.compute(newK);

	///*
	//Eigen::VectorXf deltaX  = Eigen::VectorXf::Random(3*numNonFixedVertices);
	//cg.setMaxIterations(1);
	//int i = 0;
	//do 
	//{
	//	deltaX = cg.solveWithGuess(newForce,deltaX);
	//	std::cout << i << " : " << cg.error() << std::endl;
	//	++i;
	//} while (cg.info()!=Eigen::Success && i<100);
	//*/

	////Eigen::VectorXf deltaX(3*numNonFixedVertices);
	////deltaX = cg.solve(newForce);
	////std::cout << "Error: " << cg.error() << std::endl;
	////const Eigen::VectorXf deltaX = chol.solve(newForce);

	////std::cout << "newK: " << newK << std::endl;
	////std::cout << "newForce: " << newForce << std::endl;
	////std::cout << "deltaX: " << deltaX << std::endl;

	//Eigen::VectorXf deltaX = newK.colPivHouseholderQr().solve(newForce);

	//for (int ii = 0; ii < numNonFixedVertices; ++ii)
	//{
	//	int sharedCoordIndex = nonFixedIndexes[ii];

	//	mesh->coords[sharedCoordIndex] += stepSize * deltaX.block(3*ii, 0, 3, 1);
	//}
//}