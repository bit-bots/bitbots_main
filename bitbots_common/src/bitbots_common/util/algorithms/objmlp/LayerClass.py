#-*- coding:utf-8 -*-

__author__ = 'daniel'


import numpy


class Layer:
    """
    Layer-class, used to model one layer of neurons
    """

    def __init__(self, neuronCount, parentLayerLength, label="no-name"):
        """
        Initializing layer

        :param neuronCount: Amount of neurons in this layer
        :type neuronCount: Integer

        :param parentLayerLength: Length (amount of neurons/inputs) of the parent layer
        :type parentLayerLength: Integer

        :param label: Name of the Layer
        :type label: String
        """

        self.label = str(label)
        self.neurons = []
        self.layerLength = neuronCount
        self.defaultBias = 0.1 

        # last output of all neurons on this layer
        self.lastOutput = numpy.zeros(self.layerLength)

        # last error of all neurons on this layer 
        self.error = numpy.ones(self.layerLength)

        # last innerActivation
        self.innerActivation = numpy.ones(self.layerLength)

        # weights ( rows x columns, each row stands for one neuron, each column for its weights )
        # this is always ( layerLength x parentLayerLength )
        low = 0.1
        high = 0.9
        self.weights = numpy.random.uniform(low, high, (self.layerLength, parentLayerLength))
        
        # bias ( rows x columns, each row stands for one neuron, each column for its bias )
        # this is always ( layerLength x 1 )
        self.bias = numpy.zeros(self.layerLength)
        self.bias.fill(self.defaultBias)


    def getLastOutput(self):
        return self.lastOutput


    def setLastOutput(self, nparray):
        self.lastOutput = nparray


    def getLastInnerActivation(self):
        return self.innerActivation


    def setLastInnerActivation(self, nparray):
        self.innerActivation = nparray


    def getError(self):
        return self.error


    def setError(self, nparray):
        self.error = nparray


    def getNeurons(self):
        return self.neurons


    def getAllWeightsOfNeurons(self):
        return self.weights


    def setWeights(self, nparray):
        self.weights = nparray


    def getAllBiasOfNeurons(self):
        return self.bias


    def setBias(self, nparray):
        self.bias = nparray


    def getLength(self):
        return self.layerLength


    def getLabel(self):
        return self.label