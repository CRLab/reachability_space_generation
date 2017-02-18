
// mpiCC utils.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o reach_output
// g++ utils.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o reach_output
// g++ utils.cpp -I /usr/include/hdf5/serial/ -lhdf5_serial -o reach_output

// g++  -std=c++11 utils.cpp -I /usr/include/hdf5/serial/ -lhdf5_serial -o test_interp


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
// void getClosestPoints(Eigen::MatrixXd dataset, Eigen::MatrixXd queries);
// void getClosestPoints(flann::Index<flann::L2<double> > index, Eigen::MatrixXd queries, int nn);
void getClosestPoints(flann::Index<flann::L2<double> > index, Eigen::MatrixXd queries, 
		flann::Matrix<int> indices, flann::Matrix<double> dists, int nn ) ;
void testFlaNN();

void interpolation(
    std::vector<double> query,
    std::vector<double> stepSize,
    std::vector<double> dims,
    std::vector<double> mins) ;
void testInterp() ;
void testInterp6D() ;
void testInterp2D() ;


int main(int argc, char *argv[]) {
	cout << "Hello World!" << endl;

	{
		//testInterp();
		// testInterp6D();
		testInterp2D();

		/*
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
		*/

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


flann::Index<flann::L2<double> > buildFlannIndex(Eigen::MatrixXd dataset) {
	// assert(dataset.cols() == queries.cols());

	flann::Matrix<double> datasetFlaNN;
	datasetFlaNN = convertEigen2Flann(dataset);

	cout << "datasetFlaNN: \t" << datasetFlaNN.rows << ", " << datasetFlaNN.cols << endl;
	cout << "convertEigen2Flann successful" << endl;

	// construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<double> > index(datasetFlaNN, flann::KDTreeIndexParams(4));
	index.buildIndex();
	cout << "buildIndex successful" << endl;

	return index;
}


void getClosestPoints(flann::Index<flann::L2<double> > index, Eigen::MatrixXd queries, 
		flann::Matrix<int> indices, flann::Matrix<double> dists, int nn = 3) {
	// assert(dataset.cols() == queries.cols());

	flann::Matrix<double> queriesFlaNN;
	queriesFlaNN = convertEigen2Flann(queries);

	cout << "queriesFlaNN: \t" << queriesFlaNN.rows << ", " << queriesFlaNN.cols << endl;
	cout << "convertEigen2Flann successful" << endl;

	// int nn = 3;
	// flann::Matrix<int> indices(new int[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);
	// flann::Matrix<double> dists(new double[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);

	indices = flann::Matrix<int>(new int[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);
	dists = flann::Matrix<double>(new double[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);
	cout << "creation of indices and dists successful" << endl;

	// do a knn search, using 128 checks
	index.knnSearch(queriesFlaNN, indices, dists, nn, flann::SearchParams(128));
	cout << "knnSearch successful" << endl;
	
	cout << "indices: \t" << indices.rows << ", " << indices.cols << endl;
	cout << "dists: \t" << dists.rows << ", " << dists.cols << endl;

	flann::save_to_file(indices,"resultMine.hdf5","result");
	cout << "save_to_file successful" << endl;

}

void getClosestPoints1(Eigen::MatrixXd dataset, Eigen::MatrixXd queries) {
	// assert(dataset.cols() == queries.cols());

	flann::Matrix<double> queriesFlaNN;
	queriesFlaNN = convertEigen2Flann(queries);

	cout << "queriesFlaNN: \t" << queriesFlaNN.rows << ", " << queriesFlaNN.cols << endl;
	cout << "convertEigen2Flann successful" << endl;

	int nn = 3;
	flann::Matrix<int> indices(new int[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);
	flann::Matrix<double> dists(new double[queriesFlaNN.rows*nn], queriesFlaNN.rows, nn);
	cout << "creation of indices and dists successful" << endl;

	// construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<double> > index = buildFlannIndex(dataset);

	// do a knn search, using 128 checks
	index.knnSearch(queriesFlaNN, indices, dists, nn, flann::SearchParams(128));
	cout << "knnSearch successful" << endl;

	cout << "indices: \t" << indices.rows << ", " << indices.cols << endl;
	cout << "dists: \t" << dists.rows << ", " << dists.cols << endl;

	flann::save_to_file(indices,"resultMine.hdf5","result");
	cout << "save_to_file successful" << endl;

}


void getClosestPoints0(Eigen::MatrixXd dataset, Eigen::MatrixXd queries) {
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
	flann::Matrix<int> indices(new int[queriesFlaNN.rows * nn], queriesFlaNN.rows, nn);
	flann::Matrix<double> dists(new double[queriesFlaNN.rows * nn], queriesFlaNN.rows, nn);
	cout << "creation of indices and dists successful" << endl;

	// construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<double> > index(datasetFlaNN, flann::KDTreeIndexParams(4));
	index.buildIndex();
	cout << "buildIndex successful" << endl;

	// do a knn search, using 128 checks
	index.knnSearch(queriesFlaNN, indices, dists, nn, flann::SearchParams(128));
	cout << "knnSearch successful" << endl;

	flann::save_to_file(indices, "resultMine.hdf5", "result");
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

	// getClosestPoints1(datasetMatNew, queriesMatNew);

	flann::Index<flann::L2<double> > index = buildFlannIndex(datasetMatNew);
	// getClosestPoints(index, queriesMatNew);

	flann::Matrix<int> indices;
	flann::Matrix<double> dists;
	int nn = 3;

	getClosestPoints(index, queriesMatNew, indices, dists, nn) ;

}


void printStdVector(std::vector<double> vect) {
	cout << "[ " ;
	for (int i = 0; i < vect.size() - 1; ++i)
	{
		cout << vect[i] << ", ";
	}
	cout << vect[vect.size() - 1] << " ]" << endl;
}

void testInterp2D() {
	std::vector<double> query = {0.25, 0.25};
	std::vector<double> stepSize = {0.5, 0.5};
	std::vector<double> dims = {9, 9};
	std::vector<double> mins = {0.0, 0.0};

	interpolation( query, stepSize, dims, mins);

}

void testInterp() {
	std::vector<double> query = {0.5, 0.5, 0.5};
	std::vector<double> stepSize = {1.0, 1.0, 1.0};
	std::vector<double> dims = {4, 4, 4};
	std::vector<double> mins = {0.0, 0.0, 0.0};

	interpolation( query, stepSize, dims, mins);

}

void testInterp6D() {
	std::vector<double> query = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
	std::vector<double> stepSize = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	std::vector<double> dims = {4, 4, 4, 4, 4, 4};
	std::vector<double> mins = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	interpolation( query, stepSize, dims, mins);

}

// mins: lowest corner
// dims: number of divisions in each dimensions
// stepSize: size of each division
void interpolation(
    std::vector<double> query,
    std::vector<double> stepSize,
    std::vector<double> dims,
    std::vector<double> mins)
{
	int ndims = query.size();
	int count = 1 << ndims;			// 000000
	std::vector<double> indices(count);
	std::vector<double> dists(count);

	// ndivs
	std::vector<double> ndivs(ndims);
	for (int i = 0; i < ndims; ++i)
	{
		ndivs[i] = (query[i] - mins[i]) / stepSize[i];
	}
	cout << "ndivs: \t" ;
	printStdVector(ndivs);


	// 000000
	std::vector<double> weights(ndims);
	std::vector<double> index(ndims);
	for (int i = 0; i < count; ++i)
	{
		double weight = 1;
		// for each dimension get the weight
		for (int pos = 0; pos < ndims; ++pos)
		{
			int after = (i >> pos) & 0x1;				// either zero or one
			double alpha = ndivs[pos] - int(ndivs[pos]);		// fraction between 0 and 1

			weights[pos] = ((1.0 - after) * (1.0 - alpha) + (after) * (alpha));
			weight *= weights[pos];
			index[pos] = int(ndivs[pos]) + after;
		}
		cout << i << " of " << count << endl;
		cout << "index: \t" ;
		printStdVector(index);
		cout << "weights: \t" ;
		printStdVector(weights);

		// collate index and weight
		int full_index = index[0];
		for (int dim = 1; dim <= ndims - 1; ++dim)
		{
			full_index += (full_index * dims[dim]) + index[dim];
			// const auto ind = index_x + nx * (index_y + ny * index_z);
		}

		indices[i] = full_index;
		dists[i] = weight;

		cout << "indices[i]: \t" << indices[i] << endl;
		cout << "dists[i]: \t" << dists[i] << endl << endl;
	}
}

Eigen::MatrixXd normalize_data(const Eigen::MatrixXd M, std::vector<double> stepSize)
{
	Eigen::MatrixXd M_normalized(M.rows(), M.cols());
	for (int i = 0; i < M.cols(); ++i)
	{
		M_normalized.col(i) = M.col(i) / stepSize[i];
	}
	return M_normalized;
}

void interpolation_with_nearest_neighbors(Eigen::MatrixXd dataset, Eigen::MatrixXd queries)
{
	// find the 2^ndims nearest neighbors
	// weight their values by the dist returned by NN

}