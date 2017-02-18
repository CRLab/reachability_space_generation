#!/usr/bin/python

import numpy as np

# xs = np.arange(-0.8, 0.8, 0.1)
# ys = np.arange(-0.8, 0.8, 0.1)
# zs = np.arange(0, 0.5, 0.1)
# rolls = np.arange(0, math.pi*2, 1)
# pitchs = np.arange(math.pi*-0.5, math.pi*0.5, 1)
# yaws = np.arange(0, math.pi*2, 1)
# num_combinations = len(xs)*len(ys)*len(zs)*len(rolls)*len(pitchs)*len(yaws)


xs = np.arange(0.0, 5.0, 1.0)
ys = np.arange(0.0, 5.0, 1.0)
zs = np.arange(0.0, 5.0, 1.0)
num_combinations = len(xs)*len(ys)*len(zs)
print "Looking over: " + str(num_combinations) + " combinations"


count = 0
fd = open("data.csv", "w")
for x in xs:
    for y in ys:
		for z in zs:
			data = str(count) + ", " + str(x) + ", " + str(y) + ", " + str(z) +"\n"
			fd.write(data)
			count += 1
fd.close()


## Generate 6D grid data

xs = np.arange(0.0, 5.0, 1.0)
ys = np.arange(0.0, 5.0, 1.0)
zs = np.arange(0.0, 5.0, 1.0)
rolls = np.arange(0.0, 5.0, 1.0)
pitchs = np.arange(0.0, 5.0, 1.0)
yaws = np.arange(0.0, 5.0, 1.0)
num_combinations = len(xs)*len(ys)*len(zs)*len(rolls)*len(pitchs)*len(yaws)
print "Looking over: " + str(num_combinations) + " combinations"


count = 0
fd = open("data6D.csv", "w")

for x in xs:
    for y in ys:
        for z in zs:
            for roll in rolls:
                for pitch in pitchs:
                    for yaw in yaws:
                    	data = str(count) + ", " + str(x) + ", " + str(y) + ", " + str(z) + ", " + str(roll) + ", " + str(pitch) + ", " + str(yaw) +"\n"
                    	fd.write(data)
                    	count += 1
fd.close()



## Generate 2D grid data

xs = np.arange(0.0, 5.0, 0.5)
ys = np.arange(0.0, 5.0, 0.5)
num_combinations = len(xs)*len(ys)
print "Looking over: " + str(num_combinations) + " combinations"


count = 0
fd = open("data2D.csv", "w")
for x in xs:
    for y in ys:
		data = str(count) + ", " + str(x) + ", " + str(y) +"\n"
		fd.write(data)
		count += 1
fd.close()
