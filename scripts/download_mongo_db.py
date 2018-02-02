# coding: utf-8
import reachability_db
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Upload poses to be checked for reachability.')
    parser.add_argument('INCOMPLETE_TASK_COLLECTION_NAME',
                        type=str,
                        help='config filename describing what tasks to upload to mongo. Ex: barrett.yaml, fetch22.yaml')
    parser.add_argument('FINISHED_TASK_COLLECTION_NAME',
                        type=str,
                        help='config filename describing what tasks to upload to mongo. Ex: barrett.yaml, fetch22.yaml')
    parser.add_argument('OUTPUT_FILE',
                        type=str,
                        help='Name of csv file to write mongo results to. Ex: staubli_small_reachability.csv')
    args = parser.parse_args()

    db = reachability_db.ReachabilityDB(args.INCOMPLETE_TASK_COLLECTION_NAME, args.FINISHED_TASK_COLLECTION_NAME)
    db.download_results(args.OUTPUT_FILE)

