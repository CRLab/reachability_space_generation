
 // g++ flann_example.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o flann_output
 // mpiCC flann_example.cpp -I/usr/local/ -L/usr/local/lib -lhdf5 -o flann_test

// http://stackoverflow.com/questions/18677544/hdf5-h-compile-error
// g++ flann_example.cpp -I /usr/include/hdf5/serial/ -lhdf5_serial -o flann_output



#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

#include <stdio.h>

using namespace flann;

int main(int argc, char** argv)
{
    int nn = 3;

    Matrix<float> dataset;
    Matrix<float> query;
    load_from_file(dataset, "dataset.hdf5","dataset");
    load_from_file(query, "dataset.hdf5","query");

    Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
    Matrix<float> dists(new float[query.rows*nn], query.rows, nn);

    // construct an randomized kd-tree index using 4 kd-trees
    Index<L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();                                                                                               

    // do a knn search, using 128 checks
    index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));

    flann::save_to_file(indices,"resultFlannSample.hdf5","result");

    delete[] dataset.ptr();
    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] dists.ptr();
    
    return 0;
}


// temp = h5py.File('result.hdf5', 'r')
// tmp1 = temp["result"]
// tmp1.value