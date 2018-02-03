#!/usr/bin/python

import os
import yaml
import argparse
import rospkg
import numpy as np

from tqdm import tqdm

import reachability_db


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload poses to be checked for reachability.')

    CONFIG_ROOT_DEFAULT = rospkg.RosPack().get_path('reachability_space_generation') + '/configs'
    parser.add_argument('--CONFIG_ROOT',
                        default=CONFIG_ROOT_DEFAULT,
                        type=str,
                        help='Directory containing configuration files describing what tasks to upload to mongo')

    parser.add_argument('CONFIG_FILENAME',
                        type=str,
                        help='config filename describing what tasks to upload to mongo. Ex: barrett.yaml, fetch22.yaml')

    args = parser.parse_args()

    config = yaml.load(open(os.path.join(args.CONFIG_ROOT, args.CONFIG_FILENAME)))
    for k, v in config.items():
        args.__dict__[k] = v

    args.xs = np.arange(args.x_min, args.x_max, args.x_step)
    args.ys = np.arange(args.y_min, args.y_max, args.y_step)
    args.zs = np.arange(args.z_min, args.z_max, args.z_step)

    reach_db = reachability_db.ReachabilityDB(args.incomplete_task_collection_name, args.finished_task_collection_name)

    num_combinations = len(args.xs) * len(args.ys) * len(args.zs)
    tasks = [None] * num_combinations
    count = 0
    for x in tqdm(args.xs, desc='xs'):
        for y in args.ys:
            for z in args.zs:
                task = {"count": count,
                          "x": x,
                          "y": y,
                          "z": z,
                          "status": "Incomplete"}
                tasks[count] = task
                count += 1
    print "Gathered Tasks, Doing Bulk Upload"
    ids = reach_db.bulk_add(tasks)
    print "Finished!"
