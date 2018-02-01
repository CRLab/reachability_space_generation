# coding: utf-8
from pympler import summary
from pympler import muppy
import cPickle
import os

runs = []
caches = []

mfiles = os.listdir('.')
mfiles = [x for x in mfiles if 'b.pkl' in x]
for mfile in mfiles:
    runs.append(cPickle.load(open(mfile)))
    caches.append({})
    
for run,cache in zip(runs,caches):
    for line in run:
        fname, ncalls, nsize = line
        cache[fname] = line


r10 = runs[10]
r50 = runs[50]

cache10 = caches[10]
cache50 = caches[50]

for line50 in r50:
    fname50, ncalls50, nsize50 = line50
    if fname50 not in cache10:
        print "missing from 10:" + fname50
        continue

    line10 = cache10[fname50]
    fnames10, ncalls10, nsize10 = line10
    
    #if True:
    if nsize50 != nsize10:
        print line10
        print line50
        
    if ncalls50 != ncalls10:
        print line10
        print line50

import IPython
IPython.embed()
