# coding: utf-8
import h5py
import numpy as np

temp = h5py.File('dataset.hdf5')
print temp.items()
np.savetxt('dataset.csv', temp['dataset'].value, '%g', ' ')
np.savetxt('query.csv', temp['query'].value, '%g', ' ')


# compare results
resultFlann = h5py.File('resultFlannSample.hdf5')
resultMine = h5py.File('resultMine.hdf5')
np.sum(np.sum(resultMine['result'].value - resultFlann['result'].value))


