
// g++ utils.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o reach_output
// g++ utils.cpp -I /usr/include/hdf5/serial/ -lhdf5_serial -o reach_output

// /usr/include/eigen3/Eigen

#include <iostream>
#include <fstream>
#include <sstream>
#include "vector"

#include <flann/flann.hpp>
#include <flann/io/hdf5.h>
#include <ctime>
#include <cstdlib>
// #include "nanoflann/include/nanoflann.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace std;

void load_file_as_eigen_matrix (const std::string filename, Eigen::MatrixXd &D) ;
void std_vector_2_eigen_matrix (const std::vector<std::vector<double> > vD, Eigen::MatrixXd &D);
void xyzrpy2Trans (double x, double y, double z, double roll, double pitch, double yaw, Eigen::MatrixXd &trans);
Eigen::VectorXd trans2xyzRPY (Eigen::MatrixXd &trans);

template <typename ValueType>
flann::Matrix<ValueType> convertEigen2Flann(
    const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& mEigen
);

template <typename ValueType>
flann::Matrix<ValueType> convertEigen2Flann(
    const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>& _mEigen
);
void getClosestPoints(Eigen::MatrixXd dataset, Eigen::MatrixXd queries);
void testFlaNN();


int main(int argc, char *argv[]) {
	cout << "Hello World!" << endl;

	{
		Eigen::MatrixXd D;
		std::string filename = argc > 1 ? argv[1] : "../scripts/small_data.csv";
		load_file_as_eigen_matrix (filename, D);
		// cout << D << endl;

		// test transform conversion
		Eigen::MatrixXd trans;
		xyzrpy2Trans (0, 0, 0, 0.5236, 0.5236, 0.5236, trans);
		cout << trans << endl;

		// test
		Eigen::VectorXd xyzRPY = trans2xyzRPY (trans);
		cout << xyzRPY << endl;

		// test eigen to flaNN conversion
		flann::Matrix<double> transFlaNN;
		transFlaNN = convertEigen2Flann(trans);
		cout << "size of transFlaNN: \t " << transFlaNN.rows << ", " << transFlaNN.cols << endl;
		cout << transFlaNN[0][0] << endl;

		// test eigen nearest neighbours
		testFlaNN() ;

	}
	return 0;
}


void load_file_as_eigen_matrix (const std::string filename, Eigen::MatrixXd &D) {
	std::vector<std::vector<double> > vD;
	std::string line;
	std::fstream in;
	in.open(filename.c_str());
	while (in)
	{
		std::getline(in, line);
		std::vector<double> row;
		std::stringstream stream_line(line);
		double value;
		while (stream_line >> value) row.push_back(value);
		if (!row.empty()) vD.push_back(row);
	}
	std_vector_2_eigen_matrix ( vD, D);
}


void std_vector_2_eigen_matrix (const std::vector<std::vector<double> > vD, Eigen::MatrixXd &D) {
	try {

		int n_rows = vD.size();
		cout << n_rows  << endl;
		int n_cols = vD[0].size();

		D.resize(n_rows, n_cols);

		for (int i = 0; i < n_rows; ++i)
		{
			for (int j = 0; j < n_cols; ++j)
			{
				D(i, j) = vD[i][j];
			}
		}
	} catch (const std::exception& e) {

		cout << "Error! std_vector_2_eigen_matrix!!" << endl;
	}
	// cout << D.rows() << "," << D.cols() << endl;
	// cout << D << endl;
}


void xyzrpy2Trans (double x, double y, double z, double roll, double pitch, double yaw, Eigen::MatrixXd &trans) {
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	Eigen::Matrix3d rotationMatrix = q.matrix();

	trans.resize(4, 4);
	trans.block(0, 0, 3, 3) = rotationMatrix;
	trans(0, 3) = x;
	trans(1, 3) = y;
	trans(2, 3) = z;
	trans(3, 3) = 1;
}

Eigen::MatrixXd invertTrans (Eigen::MatrixXd trans) {
	return trans.inverse();
}

Eigen::MatrixXd composeTrans (Eigen::MatrixXd &trans1, Eigen::MatrixXd &trans2) {
	return trans1 * trans2;
}

Eigen::VectorXd trans2xyzRPY (Eigen::MatrixXd &trans) {

	Eigen::VectorXd xyzRPY(6);
	xyzRPY(0) = trans(0, 3);
	xyzRPY(1) = trans(1, 3);
	xyzRPY(2) = trans(2, 3);

	Eigen::Matrix3d rotationMatrix = trans.block(0, 0, 3, 3);
	xyzRPY.segment(3, 3) = rotationMatrix.eulerAngles(2, 1, 0);

	return xyzRPY;
}


template <typename ValueType>
flann::Matrix<ValueType> convertEigen2Flann(
    const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& mEigen
)
{
	flann::Matrix<ValueType>mFlann(new ValueType[mEigen.size()], mEigen.rows(), mEigen.cols());

	for (size_t n = 0; n < mEigen.size(); ++n)
	{
		*(mFlann.ptr() + n) = *(mEigen.data() + n);
	}
	return mFlann;
}

template <typename ValueType>
flann::Matrix<ValueType> convertEigen2Flann(
    const Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>& _mEigen
)
{
	Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mEigen = _mEigen.transpose();
	return convertEigen2Flann(mEigen);
}

template <typename ValueType>
flann::Matrix<ValueType> convertVector2Flann(
    const std::vector< std::vector< ValueType> >& v
)
{
	size_t rows = v.size();
	size_t cols = v[0].size();
	size_t size = rows * cols;
	flann::Matrix<ValueType>m(new ValueType[size], rows, cols);

	for (size_t n = 0; n < size; ++n)
	{
		*(m.ptr() + n) = v[n / cols][n % cols];
	}

	return m;
}


void getClosestPoints(Eigen::MatrixXd dataset, Eigen::MatrixXd queries) {
	// assert(dataset.cols() == queries.cols());

	flann::Matrix<double> datasetFlaNN;
	datasetFlaNN = convertEigen2Flann(dataset);
	flann::Matrix<double> queriesFlaNN;
	queriesFlaNN = convertEigen2Flann(queries);

	assert(datasetFlaNN.cols == queriesFlaNN.cols);

	cout << "datasetFlaNN: \t" << datasetFlaNN.rows << ", " << datasetFlaNN.cols << endl;
	cout << "queriesFlaNN: \t" << queriesFlaNN.rows << ", " << queriesFlaNN.cols << endl;
	cout << "convertEigen2Flann successful" << endl;

	int nn = 3;
	flann::Matrix<int> indices(new int[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);
	flann::Matrix<double> dists(new double[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);
	cout << "creation of indices and dists successful" << endl;

	// construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<double> > index(datasetFlaNN, flann::KDTreeIndexParams(4));
	index.buildIndex();
	cout << "buildIndex successful" << endl;

	// do a knn search, using 128 checks
	index.knnSearch(queriesFlaNN, indices, dists, nn, flann::SearchParams(128));
	cout << "knnSearch successful" << endl;

	flann::save_to_file(indices,"resultMine.hdf5","result");
	cout << "save_to_file successful" << endl;

}

void testFlaNN() {

	Eigen::MatrixXd datasetMat;
	load_file_as_eigen_matrix ("../data/dataset.csv", datasetMat);
	cout << "datasetMat: \t" << datasetMat.rows() << ", " << datasetMat.cols() << endl;

	Eigen::MatrixXd queriesMat;
	load_file_as_eigen_matrix ("../data/query.csv", queriesMat);
	cout << "queriesMat: \t" << queriesMat.rows() << ", " << queriesMat.cols() << endl;
	// cout << D << endl;

	Eigen::MatrixXd datasetMatNew = datasetMat.transpose();
	Eigen::MatrixXd queriesMatNew = queriesMat.transpose();

	getClosestPoints(datasetMatNew, queriesMatNew);

}