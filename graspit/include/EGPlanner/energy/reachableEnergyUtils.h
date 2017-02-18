

#ifndef _reachableenergyutils_h_
#define _reachableenergyutils_h_
// g++ utils.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o reach_output
// g++ utils.cpp -I /usr/include/hdf5/serial/ -lhdf5_serial -o reach_output
// mpiCC utils.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o reach_output

// /usr/include/eigen3/Eigen

#include <iostream>
#include <fstream>
#include <sstream>
#include "vector"

#include <flann/flann.hpp>
//#include <flann/io/hdf5.h>
#include <ctime>
#include <cstdlib>
// #include "nanoflann/include/nanoflann.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace std;

void load_file_as_eigen_matrix (const std::string filename, Eigen::MatrixXd &D) ;
void std_vector_2_eigen_matrix (const std::vector<std::vector<double> > vD, Eigen::MatrixXd &D);
void xyzrpy2Trans (double x, double y, double z, double roll, double pitch, double yaw, Eigen::MatrixXd &trans);
Eigen::MatrixXd invertTrans (Eigen::MatrixXd trans);
Eigen::MatrixXd composeTrans (Eigen::MatrixXd &trans1, Eigen::MatrixXd &trans2);
Eigen::VectorXd trans2xyzRPY (Eigen::MatrixXd &trans);
flann::Index<flann::L2<double> > buildFlannIndex(Eigen::MatrixXd dataset);

template <typename ValueType>
flann::Matrix<ValueType> convertEigen2Flann(
    const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& mEigen
);

template <typename ValueType>
flann::Matrix<ValueType> convertEigen2Flann(
    const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>& _mEigen
);
//void getClosestPoints(Eigen::MatrixXd dataset, Eigen::MatrixXd queries);
void getClosestPoints(flann::Index<flann::L2<double> > index, Eigen::MatrixXd queries,
        flann::Matrix<int> indices, flann::Matrix<double> dists, int nn ) ;
void testFlaNN();
//Eigen::MatrixXd normalize_data(const Eigen::MatrixXd M, std::vector<double> stepSize);
Eigen::MatrixXd normalize_data(const Eigen::MatrixXd M, Eigen::MatrixXd stepSize);
double interpolation_nearest_neighbors(Eigen::VectorXd targetData, flann::Matrix<int> indices, flann::Matrix<double> dists);



#endif
