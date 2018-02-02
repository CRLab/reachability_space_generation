# reachability_space_generation

Define the dimensions and resolution of the reachability space, then upload the tasks to database.
- Set the ranges for sampling the robot workspace in a yaml file placed in the configs folder.
- Run `python upload_reachability_tasks.py robot.yaml` to upload all the incomplete tasks to the database.

Get multiple instances of the worker to download tasks from the database and upload the results when done.
- Create bash script in the launch folder that will launch one instance of the worker for generating the reachability space. See *launch_fetch_space_generation.sh*
- Create another script to run multiple instances of the script created in the previous step. See launch_all.sh.
- Note that you should specify the number of instances to launch based on the capacity of the machine you're using.

## Running
```
source ../../../devel/setup.bash
./launch_all.sh
```


## Download completed reachability space from database
- Run download_mongo_db.py with three positional args:
    - INCOMPLETE_TASK_COLLECTION_NAME e.g robot_tasks_to_do
    - FINISHED_TASK_COLLECTION_NAME e.g. robot_tasks_finished
    - OUTPUT_FILE e.g. robot_dense_reachability.csv

```
python download_mongo_db.py robot_tasks_to_do robot_tasks_finished robot_dense_reachability.csv
```