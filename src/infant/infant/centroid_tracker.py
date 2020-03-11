from time import time

import numpy as np
from scipy.spatial import distance as dist


class CentroidTracker():
    def __init__(self, half_life, timeout):
        self.decay = 2**(-1 / half_life)
        self.timeout = timeout
        self.reset()

    def register(self, centroid, area, t):
        # [centroid, area, spawn_time, last_seen]
        self.objects[self.nextObjectID] = [centroid, area, t, t]
        self.nextObjectID += 1

    def unregister(self, objectID):
        del self.objects[objectID]

    def update(self, rects):
        t = time()
        inputCentroids = np.array([[(rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2] for rect in rects], dtype=int)
        inputAreas = np.array([(rect[2] - rect[0]) * (rect[3] - rect[1]) for rect in rects])
        if not self.objects:
            for centroid, area in zip(inputCentroids, inputAreas):
                self.register(centroid, area, t)
        elif not len(rects):
            for objectID in list(self.objects.keys()):
                if t - self.objects[objectID][3] > self.timeout:
                    self.unregister(objectID)
        else:
            objectIDs, objects = zip(*self.objects.items())
            D = dist.cdist(np.array([val[0] for val in objects]), inputCentroids)
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            usedRows = set()
            usedCols = set()
            for row, col in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue
                objectID = objectIDs[row]
                self.objects[objectID][0] = inputCentroids[col]
                self.objects[objectID][1] = inputAreas[col]
                self.objects[objectID][3] = t
                usedRows.add(row)
                usedCols.add(col)
            for row in set(range(0, D.shape[0])).difference(usedRows):
                objectID = objectIDs[row]
                if t - self.objects[objectID][3] > self.timeout:
                    self.unregister(objectID)
            for col in set(range(0, D.shape[1])).difference(usedCols):
                self.register(inputCentroids[col], inputAreas[col], t)
        return [[obj[0], obj[1], self.decay**(t - obj[2])] for obj in self.objects.values() if obj[3] == t]

    def reset(self):
        self.nextObjectID = 0
        self.objects = {}
