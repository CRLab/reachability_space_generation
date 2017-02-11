#!/usr/bin/python
import os
import fnmatch

location = '/home/baxter/.ros/'
filenames = os.listdir(location)
checked_file = []

for file in filenames:
    if fnmatch.fnmatch(file, '*.csv'):
        checked_file.append(file)

checked_file.sort()


with open('/home/baxter/ros/reachability_data/src/grasp_reachability_planning/scripts/merged_data.csv', 'a') as outfile:
    for fname in checked_file:
        with open(location+fname) as infile:
            for line in infile:
                outfile.write(line)
