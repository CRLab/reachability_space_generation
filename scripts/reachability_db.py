from pymongo import MongoClient, ASCENDING, DESCENDING


class ReachabilityDB(object):
    def __init__(self):
        self.mongo_hostname = "mainland.cs.columbia.edu"
        self.mongo_url = "mongodb://{}:27017".format(self.mongo_hostname)

        self.client = MongoClient(self.mongo_url)
        self.collection = self.client.reachability.staubli_tasks
        self.collection.create_index([("count", DESCENDING)],
                                     unique=True)

    def bulk_add(self, tasks):
        ids = self.collection.insert(tasks)
        return ids

    def get_task(self):
        task = self.collection.find_and_modify(query={"status": "Incomplete"},
                                               just_one=True,
                                               update={"$set": {'status': "In Progress"}})
        return task

    def record_task_result(self, count, reachable):
        self.collection.find_and_modify(query={"count": count},
                                        just_one=True,
                                        update={"$set": {'status': "Finished", 'reachable': reachable}})
