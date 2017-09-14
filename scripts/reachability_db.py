from pymongo import MongoClient, ASCENDING, DESCENDING


class ReachabilityDB(object):
    def __init__(self):
        self.mongo_hostname = "mainland.cs.columbia.edu"
        self.mongo_url = "mongodb://{}:27017".format(self.mongo_hostname)

        self.client = MongoClient(self.mongo_url)
        self.collection = self.client.reachability.staubli
        self.collection.create_index([("x", DESCENDING),
                                      ("y", DESCENDING),
                                      ("z", DESCENDING),
                                      ("roll", DESCENDING),
                                      ("pitch", DESCENDING),
                                      ("yaw", DESCENDING)],
                                     unique=True)

    def add(self,
            x, y, z, roll, pitch, yaw, reachable):

        result = {
                  "x": x,
                  "y": y,
                  "z": z,
                  "roll": roll,
                  "pitch": pitch,
                  "yaw": yaw,
                  "reachable": reachable}

        self.collection.insert(result)
        

    def get_data(self):
        pass
