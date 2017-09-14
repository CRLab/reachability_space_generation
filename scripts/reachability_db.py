from pymongo import MongoClient, ASCENDING, DESCENDING


class ReachabilityDB(object):
    def __init__(self):
        self.mongo_hostname = "mainland.cs.columbia.edu"
        self.mongo_url = "mongodb://{}:27017".format(self.mongo_hostname)

        self.client = MongoClient(self.mongo_url)
        self.bulk_tasks_collection = self.client.reachability.bulk_staubli_tasks
        self.bulk_tasks_collection.create_index([("count", DESCENDING)],
                                     unique=True)

        self.finished_results_collection = self.client.reachability.finished_staubli_tasks

    def bulk_add(self, tasks):
        ids = self.bulk_tasks_collection.insert(tasks)
        return ids

    def get_bulk_task(self):
        task = self.bulk_tasks_collection.find_and_modify(query={"status": "Incomplete"},
                                               just_one=True,
                                               update={"$set": {'status': "In Progress"}})
        return task

    def record_task_result(self, count, tasks_to_be_uploaded):

        # upload the completed results
        ids = self.finished_results_collection.insert(tasks_to_be_uploaded)

        # mark the bulk task as finished
        self.bulk_tasks_collection.find_and_modify(query={"count": count},
                                        just_one=True,
                                        update={"$set": {'status': "Finished"}})



        return ids
