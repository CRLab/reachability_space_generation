# coding: utf-8
import reachability_db

db = reachability_db.ReachabilityDB()

f = open("temp.csv", 'w')

cursor = db.collection.find()
for i, e in enumerate(cursor):
    f.write("{} {} {} {} {} {} {} {}\n".format(i, e['x'], e['y'], e['z'], e['roll'], e['pitch'], e['yaw'], e['reachable']))
    
f.close()

