#!/usr/bin/python

import os
import yaml
import argparse

from tqdm import tqdm

import reachability_db

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload poses to be checked for reachability.')
    parser.add_argument('--CONFIG_ROOT',
                        default="/home/iakinola/ros/reachability_space_ws/src/reachability_space_generation/configs",
                        type=str,
                        help='Directory containing configuration files describing what tasks to upload to mongo')

    parser.add_argument('--CONFIG_FILENAME',
                        default="barrett.yaml",
                        type=str,
                        help='configuration filename describing what tasks to upload to mongo')

    args = parser.parse_args()

    config_filepath = os.path.join(args.CONFIG_ROOT, args.CONFIG_FILENAME)
    config = yaml.load(open(config_filepath))
    for k, v in config.items():
        args.__dict__[k] = v

    reach_db = reachability_db.ReachabilityDB()

    num_combinations = len(args.xs) * len(args.ys) * len(args.zs) * len(args.rolls) * len(args.pitchs) * len(args.yaws)
    tasks = [None] * num_combinations
    count = 0
    for x in tqdm(args.xs, desc='xs'):
        for y in args.ys:
            for z in args.zs:
                for roll in args.rolls:
                    for pitch in args.pitchs:
                        for yaw in args.yaws:
                            task = {"count": count,
                                      "x": x,
                                      "y": y,
                                      "z": z,
                                      "roll": roll,
                                      "pitch": pitch,
                                      "yaw": yaw,
                                      "status": "Incomplete"}
                            tasks[count] = task
                            count += 1
    print "Gathered Tasks, Doing Bulk Upload"
    ids = reach_db.bulk_add(tasks)
    print "Finished!"
