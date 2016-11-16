# -*- coding: utf-8 -*-

"""
DBSCAN
^^^^^^

Module for filtering data via DBSCAN algorithm.

History:
''''''''

* 07.01.15: Created (Daniel Speck)

"""

from collections import deque
import numpy as np




class DBScan():
    """
    DBSCAN class

    This class delivers a DBSCAN object which filters two-dimensional input data via
    a slightly customized DBSCAN algorithm.
    """

    # fields - only for initialization purposes
    data = None
    dataCount = None

    clusters = None
    clusterCount = None

    filteredData = None

    minPoints = None
    epsilon = None

    datamap = None



    def __init__(self, datax, datay, minPoints, epsilon):
        """
            Creates a dbscan object to filter given datasets

            :param datax: 1D numpy array of x-input data
            :param datay: 1D numpy array of y-input data
            :param minPoints: integer, minimum neighborhood points for dbscan
            :param epsilon: float, minimum epsilon value for dbscan

            :return: x,y (x-values-array, y-values-array)
        """
      

        ######################
        # exception handling #
        ######################

        # is datax set correctly?
        if not(type(datax) == np.ndarray) or not(vars().has_key("datax")):
            raise Exception("datax has to be set and be of type numpy.ndarray")

        # is datay set correctly?
        if not(type(datay) == np.ndarray) or not(vars().has_key("datay")):
            raise Exception("datay has to be set and be of type numpy.ndarray")

        # shapes of datax and datay have to match
        if datax.shape != datay.shape:
            raise Exception(
                    "shapes of datax and datay have to match."
                    + "\nshape of datax: " + str(datax.shape)
                    + "\nshape of datay: " + str(datay.shape)
                    )

        # is minPoints set correctly?
        if not(type(minPoints) == int) or not(vars().has_key("minPoints")):
            raise Exception("minPoints has to be set and be of type int")
        
        # is epsilon set correctly?
        if not(type(epsilon) == float) or not(vars().has_key("epsilon")):
            raise Exception("epsilon has to be set and be of type float")


        ########################
        # initialization stuff #
        ########################


        # initialize input data
        # row 0 contains datax, row 1 datay
        # each column represents one dataset of x,y-values
        self.data = np.array([datax,datay])

        # amount of datasets (input)
        self.dataCount = self.data.shape[1]

        # initialize filtered data
        self.filteredData = False

        # initialize amount of clusters
        self.clusterCount = 0

        # initialize clusters
        # contains all found clusters
        self.clusters = deque()

        # initialize minPoints (to build a cluster)
        self.minPoints = minPoints

        # initialize epsilon (maximum range for neighbors)
        self.epsilon = epsilon

        # initialize a datamap for each point
        # each value represents the following:
        # 0: point is not visited (not calculated yet)
        # 1: point is core point
        # 2: point is density-reachable
        # 3: point is noise
        #
        # e.g. 'datamap[2][5] = 1' means
        # the point 'data[2][5]' is a core point of a cluster
        #
        self.datamap = np.zeros(self.data.shape[1])

        # filter data 
        self.__filter()



    def getFilteredData(self):
        """
        Returns the DBSCAN-filtered data sets

        :return: A numpy.ndarray representing the DBSCAN-filtered data sets
        """
        return self.filteredData[0], self.filteredData[1]



    def __filter(self):
        """
        Core filter algorithm.
        Iterates through the data sets and marks them as either core, density-reachable or noise.
        Also detects and builds clusters.
        """

        # iterate over every dataset
        for i in xrange(0, self.dataCount):

            # if point is already visited: skip execution
            if self.datamap[i] != 0:
                continue

            # indexes of neighborhood points
            neighborPoints = self.__regionQuery(i)

            # number of neighborhood points
            neighborCount = len(neighborPoints)


            if neighborCount < self.minPoints:
                # mark point as noise
                self.datamap[i] = 3
            else:
                # found new cluster
                self.clusterCount += 1
                self.__expandCluster(neighborPoints)

        # get filteredKeys from datamap
        filteredKeys = np.where(self.datamap != 3)
        filteredKeys = filteredKeys[0]
        
        # initialize filteredData with zeros
        self.filteredData = np.zeros((2, filteredKeys.shape[0]))

        # set filteredData
        for i in xrange(0, len(filteredKeys)):
            self.filteredData[0][i] = self.data[0][filteredKeys[i]] # x values
            self.filteredData[1][i] = self.data[1][filteredKeys[i]] # y values



    def __expandCluster(self, neighborPoints):
        """
        Expands a newly found cluster.

        :param neighborPoints: Neighbor points (data sets) of the current cluster
        """

        # initialize a new cluster
        newCluster = deque()
        
        # container / iterator
        contNeighborPoints = list(neighborPoints)    

        for pointIndex in contNeighborPoints:
            
            # if point is not visited
            if self.datamap[pointIndex] == 0 or self.datamap[pointIndex] == 3:
                
                # current neighbors of current loop point
                curNeighbors = self.__regionQuery(pointIndex)

                # append new neighborhood to iterator
                [contNeighborPoints.append(elem) for elem in curNeighbors]

                if len(curNeighbors) >= self.minPoints:
                    newCluster.append(pointIndex)
                    self.datamap[pointIndex] = 1  # mark it as core point

                elif len(curNeighbors) > 0:
                    self.datamap[pointIndex] = 2  # mark it as density-reachable

        # append the new cluster to our clusters buffer
        self.clusters.append(newCluster)



    def __regionQuery(self, i):
        """
        Searches for neighboring points for a given data set i in a distance <= epsilon

        ::param i: one data set / point, in whose epsilon region should be searched

        :return: a numpy array containing the indexes of all found neighboring points
        """

        # prepare a buffer object for the indexes of eventually found neighboring points
        regionIndexes = deque()

        # get a numpy iterator with C indexing-style
        npiter = np.nditer([self.data[0], self.data[1]], flags=["c_index"])

        # iterate through the numpy iterator
        for x, y in npiter:
            # if the currently processed point of the iterator is in a circle
            # with the radius of self.epsilon it is a neighboring point
            if np.sqrt(np.square(x - self.data[0][i]) + np.square(y - self.data[1][i])) < self.epsilon:
                regionIndexes.append(npiter.index)  # append the points index to the found neighbors

        points = np.array(regionIndexes)                

        # this returns our array (tuple is not necessary)
        return points