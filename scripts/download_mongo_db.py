# coding: utf-8
import reachability_db
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Upload poses to be checked for reachability.')
    parser.add_argument('--OUTPUT_FILE',
                        default="staubli_small_reachability.csv",
                        type=str,
                        help='Name of csv file to write mongo results to.')
    args = parser.parse_args()

    db = reachability_db.ReachabilityDB()
    db.download_results(args.OUTPUT_FILE)

