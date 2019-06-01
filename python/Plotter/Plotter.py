from mpl_toolkits.mplot3d import Axes3D
from sklearn import linear_model
from sklearn.decomposition import PCA

import matplotlib
import matplotlib.pyplot as plt

import pandas as pd

import csv
import os

class Plotter:
    """

    """
    def __init__(self, file_path):
       self.data_hash = pd.read_csv(file_path, names=['ts', 'x', 'y', 'pt_mapd'])

       fig = plt.figure()
       # ax.scatter(self.data_hash['pt_mapd'], [0] * self.data_hash.shape[0], marker='x')
       ax = fig.add_subplot(111, projection='3d')
       ax.scatter(self.data_hash['x'], self.data_hash['y'], self.data_hash['pt_mapd'], marker='.')

       # Check for duplicates
       duplicate_rows = self.data_hash[self.data_hash.duplicated(['pt_mapd'])]
       print(row for row in duplicate_rows)

       ax.set_xlabel('X')
       ax.set_ylabel('Y')
       ax.set_zlabel('Mapped Pts')

       plt.show()


if __name__ == "__main__":

    print ("Current path : {}".format(os.getcwd()))
    Plotter('../../src/distributed_slam/tests/logs/test-env/mapped_pts.txt')
