
 // g++ utils.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o reach_output

// /usr/include/eigen3/Eigen

#include <iostream>
#include <fstream>
#include <sstream>
#include "vector"

#include <flann/flann.hpp>
// #include <flann/io/hdf5.h>
#include <ctime>
#include <cstdlib>
#include "nanoflann/include/nanoflann.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace std;


void std_vector_2_eigen_matrix (const std::vector<std::vector<double> > vD, Eigen::MatrixXd &D);
void xyzrpy2Trans (double x, double y, double z, double roll, double pitch, double yaw, Eigen::MatrixXd &trans);
Eigen::VectorXd trans2xyzRPY (Eigen::MatrixXd &trans);

	int main(int argc, char *argv[])
	{
		cout << "Hello World!" <<endl;

		{
    Eigen::MatrixXd D;
    std::vector<std::vector<double> > vD;
    std::string line;
    std::fstream in;
    in.open(argc>1?argv[1]:"../scripts/small_data.csv");
    while(in)
    {
      std::getline(in, line);
      // cout << line << endl;
      std::vector<double> row;
      std::stringstream stream_line(line);
      double value;
      while(stream_line >> value) row.push_back(value);
      if(!row.empty()) vD.push_back(row);
    }
    std_vector_2_eigen_matrix ( vD, D);

    // test transform conversion
    Eigen::MatrixXd trans;
    xyzrpy2Trans (0, 0, 0, 0.5236, 0.5236, 0.5236, trans);
    cout << trans << endl;

    //
    Eigen::VectorXd xyzRPY = trans2xyzRPY (trans);
    cout << xyzRPY << endl;


    // if(!igl::list_to_matrix(vD,D)) return EXIT_FAILURE;
    // assert(D.cols() == 6 && "pwn file should have 6 columns");
    // P = D.leftCols(3);
    // N = D.rightCols(3);
  }
		return 0;
	}


	void std_vector_2_eigen_matrix (const std::vector<std::vector<double> > vD, Eigen::MatrixXd &D) {
		int n_rows = vD.size();
		int n_cols = vD[0].size();

		D.resize(n_rows, n_cols);

		for (int i = 0; i < n_rows; ++i)
		{
			for (int j = 0; j < n_cols; ++j)
			{
				D(i,j) = vD[i][j];
			}
		}
      // cout << D.rows() << "," << D.cols() << endl;
      // cout << D << endl;
	}


	void xyzrpy2Trans (double x, double y, double z, double roll, double pitch, double yaw, Eigen::MatrixXd &trans){
		Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

		Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
		Eigen::Matrix3d rotationMatrix = q.matrix();

		trans.resize(4, 4);
		trans.block(0,0,3,3) = rotationMatrix;
		trans(0,3) = x;
		trans(1,3) = y;
		trans(2,3) = z;
		trans(3,3) = 1;
	}

	Eigen::MatrixXd invertTrans (Eigen::MatrixXd trans){
		return trans.inverse();
	}

	Eigen::MatrixXd composeTrans (Eigen::MatrixXd &trans1, Eigen::MatrixXd &trans2){
		return trans1 * trans2;		
	}

	Eigen::VectorXd trans2xyzRPY (Eigen::MatrixXd &trans){

		Eigen::VectorXd xyzRPY(6);
		xyzRPY(0) = trans(0,3);
		xyzRPY(1) = trans(1,3);
		xyzRPY(2) = trans(2,3);

		Eigen::Matrix3d rotationMatrix = trans.block(0,0,3,3);
		xyzRPY.segment(3,3) = rotationMatrix.eulerAngles(2, 1, 0);

		return xyzRPY;		
	}

	void eigen2Flann (Eigen::MatrixXd &eigMat, flann::Matrix<float> &flannMat){

	}


// 	template <typename ValueType>
// flann::Matrix<ValueType> convertEigen2Flann(
// const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&amp; mEigen
// )
// {
// flann::Matrix<ValueType>mFlann(new ValueType[mEigen.size()], mEigen.rows(), mEigen.cols());
 
// for(size_t n = 0; n < mEigen.size(); ++n)
// {
// *(mFlann.ptr()+n) = *(mEigen.data()+n);
// }
// return mFlann;
// }
 
// template <typename ValueType>
// flann::Matrix<ValueType> convertEigen2Flann(
// const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>&amp; _mEigen
// )
// {
// Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mEigen = _mEigen.transpose();
// return convertEigen2Flann(mEigen);
// }

	void getClosestPoints(Eigen::MatrixXd dataset, Eigen::VectorXd query){
		assert(dataset.cols() == query.size());

		int dim = query.size();
		vector<int> query_pt(query.data(), query.data() + query.size());
		// vector<int> vec(mat.data(), mat.data() + mat.rows() * mat.cols());

		typedef KDTreeEigenMatrixAdaptor< Eigen::Matrix<double,double,double> >  my_kd_tree_t;
		my_kd_tree_t   mat_index(dim /*dim*/, mat, 10 /* max leaf */ );

		// KDTreeEigenMatrixAdaptor< Eigen::MatrixXd >  mat_index(dim , dataset, 10 /* max leaf */ );

		mat_index.index->buildIndex();

		// do a knn search
	const size_t num_results = 3;
	vector<size_t>   ret_indexes(num_results);
	vector<double> out_dists_sqr(num_results);

	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
	mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	for (size_t i=0;i<num_results;i++)
		std::cout << "ret_index["<<i<<"]=" << ret_indexes[i] << " out_dist_sqr=" << out_dists_sqr[i] << endl;

	}