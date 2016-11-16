#-*- coding:utf-8 -*-

__author__ = 'Daniel Speck'

"""
NeuralNetworkClass
^^^^^^^^^^^^^^^^^^

Module for creating, teaching and calculating an artificial neural network.

History:
''''''''

* 18.02.15: Created (Daniel Speck)

"""

import LayerClass

import json
import base64
import numpy
import sys

class NeuralNetwork:
    """
    NeuralNetwork class

    This class creates, teaches and calculates data via an artificial neural network.
    """

    def __init__(self, in_input, hiddenLayerList, outputLayerLength, label="no-name"):
        """
        Initializing of the artificial neural network

        :param in_input: Length of input layer
        :type in_input: int

        :param hiddenLayerList: List of hidden layers
        :type hiddenLayerList: list[int]

        :param outputLayerLength: Number of output Neurons
        :type outputLayerLength: int



        ### Examples ###

        in_input = 2                # Input layer length: 2 neurons
        hiddenLayerList = [2,3,2]   # Would create 3 Hidden Layers with 2, 3 and 2 Neurons
        outputLayerLength = 3       # Creates an output layer of length 3 (3 neurons)
        """

        self.__checkForErrorsInit(in_input, hiddenLayerList, outputLayerLength)

        # set the label of the net
        self.label = str(label)

        # initialize the input layer data and length
        self.inputData = None
        self.inputLayerLength = in_input

        # initialize the exptected output values for each entry in inputLayer
        # has to be set in teach method
        self.outExpected = None

        # initialize the output of the network
        self.output = None

        # initialize hiddenLayer
        self.hiddenLayer = []

        # insert the length of input layer to the beginning of the hiddenLayerList
        # this is needed to calculate the weights for layer 1 of the hidden layers
        hiddenLayerList.insert(0, self.inputLayerLength)

        # create all hidden layers in one loop
        for i in range(1, len(hiddenLayerList)):
            # append one complete layer to hiddenLayer
            # Layer( neuronCount, parentLayerLength, label )
            self.hiddenLayer.append(LayerClass.Layer(hiddenLayerList[i], hiddenLayerList[i-1], i))

        # create the output layer
        # Layer( neuronCount, parentLayerLength, label )
        self.outputLayer = LayerClass.Layer(outputLayerLength, self.hiddenLayer[len(self.hiddenLayer)-1].getLength(), "Output")

        # make all layers easy accessable
        self.hiddenAndOutputLayer = []
        self.hiddenAndOutputLayer += self.hiddenLayer
        self.hiddenAndOutputLayer.append(self.outputLayer)


    def teach(self, inputData, expectedOutputData, iterations=1000, epsilon=0.2):
        """
        Trains the net with a given set of inputData and matching expectedOutputData (supervised learning)

        :param iterations: number of learning steps to make
        :type iterations: int

        :param epsilon: learning rate
        :type epsilon: float

        :param inputData: InputLayer -> List of lists which hold input value/data for
                                    each input neuron

                                    Must have same length as out_expected

                        Example for XOR-Problem:
                        [ [0,0], [0,1], [1,0], [1,1] ]

        :param expectedOutputData: List containing lists which hold expected output values/data
                                   Must have same length as in_input

                            Example for XOR-Problem:
                            [ [0], [1], [1], [0] ]
        """

        # check input data
        self.__checkForErrorsTeach(inputData, expectedOutputData)

        # set inputData and ExpectedOutput
        self.inputData = inputData
        self.outExpected = expectedOutputData

        # teaching loop
        for i in range(0, iterations):
            # set input and expectedOutput
            randomSelect = numpy.random.randint(0, len(self.inputData))

            input = numpy.array(self.inputData[randomSelect])
            self.currentOutExpected = numpy.array(self.outExpected[randomSelect])

            # feedforward
            self.output = self.__feedforward(input)

            # calculate errors
            self.__updateErrors()

            # update weights
            self.__updateWeights(input, epsilon)

            # update bias
            self.__updateBias(epsilon)


    def __updateErrors(self):
        """
        Calculates the new errors for each layer/neuron and saves this calculated
        data in fields/buffers.
        """

        debug = False

        # calculate the erros

        # for index in range ( endOfTheList, beginningOfTheList, decrement index by 1 )
        # so this loop starts at the end of all layers (outputLayer)
        # and iterates its way to the top
        for i in range(len(self.hiddenAndOutputLayer) - 1, -1, -1):
            # error for output layer --- first iteration
            if i == (len(self.hiddenAndOutputLayer)-1):
                self.outputLayer.setError( self.currentOutExpected - self.output )
                if debug:
                    self.__print("output error:", self.outputLayer.getError())

            # error for hidden layers
            else:
                # get current layer to update its error
                currentLayer = self.hiddenAndOutputLayer[i]

                # get the underlyingLayer (lambda+1) for backpropagation/calculation
                underlyingLayer = self.hiddenAndOutputLayer[i+1]
                # calculate new error
                currentLayer.setError(
                       self.transferFunctionDeriv( currentLayer.getLastInnerActivation() )
                        *
                        numpy.dot(
                            underlyingLayer.getError(),
                            underlyingLayer.getAllWeightsOfNeurons()
                            )
                        )

                # only for debugging purposes
                if debug:
                    print("")
                    self.__print("layer", i)
                    self.__print("activation",
                            currentLayer.getLastInnerActivation())
                    self.__print("_deriv(activation)",
                            self.transferFunctionDeriv(currentLayer.getLastInnerActivation()))
                    self.__print("underlyingLayer.error",
                            underlyingLayer.getError())
                    self.__print("underlyingLayer.weights",
                            underlyingLayer.getAllWeightsOfNeurons())
                    self.__print("error x weights",
                            numpy.dot(underlyingLayer.getError(),underlyingLayer.getAllWeightsOfNeurons()) )
                    self.__print("new error (acti * (error x weights))",
                            currentLayer.getError())


    def __print(self, label, item):
        """
        Prints a given items label and value.

        :param label: label of the item to be printed
        :param item: value of the itme to be printed
        """
        print(str(label) + " :\n" + str(item))


    def __updateBias(self, epsilon):
        """
        Update the bias of the net

        :param epsilon: learning rate
        """

        for layer in self.hiddenAndOutputLayer:

            layer.setBias(layer.getAllBiasOfNeurons() + (epsilon * layer.getError()))


    def __updateWeights(self, u_input, epsilon):
        """
        Updates the weights of the net

        :param u_input: input data
        :param epsilon: learning rate
        """

        debug = False

        # for each layer update the weights at once
        for key, layer in enumerate(self.hiddenAndOutputLayer):

            # for the first hidden layer it is epsilon * error * input
            if key == 0:
                layer.setWeights(
                        layer.getAllWeightsOfNeurons()
                        +
                        (
                         epsilon
                         *
                         (
                          layer.getError()[numpy.newaxis].T
                          *
                          u_input
                         )
                        )
                        )

                # only for debugging purposes
                if debug:
                    print("")
                    self.__print("weights", layer.getAllWeightsOfNeurons())
                    self.__print("epsilon", epsilon)
                    self.__print("layer error", layer.getError()[numpy.newaxis].T)
                    self.__print("input", u_input)
                    self.__print("error x input", layer.getError()[numpy.newaxis].T * u_input)
                    self.__print("epsilon * (error x input)", epsilon
                             *
                             (
                              layer.getError()[numpy.newaxis].T
                              *
                              u_input
                             ))
                    self.__print("new weights: ", layer.getAllWeightsOfNeurons()
                            +
                            (
                             epsilon
                             *
                             (
                              layer.getError()[numpy.newaxis].T
                              *
                              u_input
                             )
                            ))

            # for all other hidden layers and output layer the calculation
            # is epsilon * error * output_of_parent_layer
            else:
                layer.setWeights(
                        layer.getAllWeightsOfNeurons()
                        +
                        (
                         epsilon
                         *
                         (
                             layer.getError()[numpy.newaxis].T
                             *
                             self.hiddenAndOutputLayer[key-1].getLastOutput()
                         )
                        )
                        )

                # only for debugging purposes
                if debug:
                    print("")
                    self.__print("weights", layer.getAllWeightsOfNeurons())
                    self.__print("epsilon", epsilon)
                    self.__print("layer error", layer.getError()[numpy.newaxis].T)
                    self.__print("input (parent output)", self.hiddenAndOutputLayer[key-1].getLastOutput())
                    self.__print("error x input", layer.getError()[numpy.newaxis].T * self.hiddenAndOutputLayer[key-1].getLastOutput())
                    self.__print("epsilon * (error x input)", epsilon
                             *
                             (
                              layer.getError()[numpy.newaxis].T
                              *
                              self.hiddenAndOutputLayer[key-1].getLastOutput()
                             ))
                    self.__print("new weights: ", layer.getAllWeightsOfNeurons()
                            +
                            (
                             epsilon
                             *
                             (
                              layer.getError()[numpy.newaxis].T
                              *
                              self.hiddenAndOutputLayer[key-1].getLastOutput()
                             )
                            ))


    def __feedforward(self, f_input):
        """
        Does a feedforward activation of the whole net

        :param f_input: represents the input data, has to be a numpy array matching the dimension of InputLayer
        """

        ######################
        ### initialization ###
        ######################

        # set last_out to current input
        last_out = f_input

        #######################################
        ### start calculating - feedforward ###
        #######################################

        # feedforward - loop through all layers    
        for key, layer in enumerate(self.hiddenAndOutputLayer):

            layer = self.hiddenAndOutputLayer[key]
            parentLayer = self.hiddenAndOutputLayer[key-1]

            # calculate inner activation without bias
            # numpy.dot of all neuron weights of the current layer and the output of the parent layer
            innerActivation = numpy.dot(layer.getAllWeightsOfNeurons(), last_out)

            # add the bias of each neuron to the innerActivation
            innerActivation += layer.getAllBiasOfNeurons()

            # standardize innerActivation
            innerActivation /= parentLayer.getLength()

            # save new innerActivation values in layer
            layer.setLastInnerActivation(innerActivation)

            # calculate new output of the current layer
            # this is used as input for the next layer in the next iteration
            # or as output for the output layer when the loop is finished
            last_out = self.transferFuction(innerActivation)

            # save new output values in layer
            layer.setLastOutput(last_out)

        return layer.getLastOutput()


    def calculate(self, c_input, expOut = None):
        """
        Calculates the output for some given input (feedforward activation)
        
        :param c_input: a list, containing the input data for input layer

        :param expOut: optional parameter, represents the expected output for a given input

        :return: a dictionary {"output", "input", "expectedOutput"} of calculated data
        """


        ######################
        ### error checking ###
        ######################

        # check if c_input is set correctly
        if not ((type(c_input) is list) and (len(c_input) == self.inputLayerLength)):
            sys.exit("Error: c_input has to be a list containing the input data of length "
                     + str(self.inputLayerLength))

        # calculate & set the output of the net
        self.output = self.__feedforward(numpy.array(c_input))

        return {"output": self.output, "input": c_input, "expectedOutput": expOut}

    def transferFuction(self, x):
        """
        Transfer function

        :param x: input value which has to be normalized by the transfer function
        """

        return numpy.tanh(x)


    def transferFunctionDeriv(self, x):
        """
        Derivate of the transfer function

        :param x: input value which has to be normalized by the derivated transfer function
        """

        return 1 - numpy.power(numpy.tanh(x), 2)


    def printNetWeights(self):
        """
        Prints the nets weights for debugging purposes
        """

        print("######################################################")
        print("     neural net: " + self.label)
        print("######################################################")
        print("")
        print("###############")
        print("### weights ###")
        print("###############")

        print("")

        print("-> " + str(len(self.hiddenAndOutputLayer)) + " layers total (hidden + output layer, input is not counted)")
        print("-> layer " + str(len(self.hiddenAndOutputLayer)-1) + " is the output layer")
        print("")

        for key, layer in enumerate(self.hiddenAndOutputLayer):

            print("--- Layer: " + str(key)
                    + " --- (" + str(layer.getLength())
                    + " neurons total | each row represents the weights of one neuron)")
            for neuronweight in layer.getAllWeightsOfNeurons():
                print(neuronweight)
            print("")


    def __encodeNumpyArray(self, nparray):
        """
        Encodes a numpy array.
        The Array is encoded as a list containing its datatype, data (base64-encoded) and shape.

        :param nparray: numpy ndarray to encode

        :return: json dumped string (encoded numpy array)
        """
        return json.dumps([str(nparray.dtype), base64.b64encode(nparray), nparray.shape])


    def __decodeNumpyArray(self, encStr):
        """
        Decodes a numpy array.
        The Array has to be encoded as a list containing the arrays datatype, data (base64-encoded) and shape.

        :param encStr: an encoded string of numpy ndarray type, data and shape

        :return: json dumped string (encoded numpy array)
        """

        # encode the json dump
        enc = json.loads(encStr)

        # build the numpy data type
        dataType = numpy.dtype(enc[0])

        # decode the base64 encoded numpy array data and create a new numpy array with this data & type
        dataArray = numpy.frombuffer(base64.decodestring(enc[1]), dataType)

        # if the array had more than one data set it has to be reshaped
        if len(enc) > 2:
            return dataArray.reshape(enc[2])   # return the reshaped numpy array containing several data sets
        else:
            return dataArray                   # return single data set numpy array


    def saveWeightsAndBias(self, path=""):
        """
        Saves the current weights and bias

        :param path: optional parameter defining the path to the save destination.
        """

        copy = self.hiddenAndOutputLayer
        dataStr = ""

        with open(path + "netlog_WeightsAndBias.log", "w") as log:
            # save input layer information
            dataStr += "+++input_layer_size===>" + str(self.inputLayerLength) + "\n"

            # loop through layers
            for layer in copy:
                dataStr += "###layer===>" + layer.getLabel() + "###\n"
                dataStr += "w===>" + self.__encodeNumpyArray(layer.getAllWeightsOfNeurons()) + "\n"
                dataStr += "b===>" + self.__encodeNumpyArray(layer.getAllBiasOfNeurons()) + "\n"
                dataStr += "s===>" + str(layer.getLength()) + "\n"

            # write data
            log.write(dataStr)


    def loadWeightsAndBias(self, path=""):
        """
        Loads weights and bias from a file

        :param path: optional parameter defining the path to the load destination.
        """

        logfile = open(path + "netlog_WeightsAndBias.log", "r")

        # buffers for looping
        curLayerLabel = None
        curLayerWeights = None
        curLayerBias = None
        curLayerSize = None

        # size of the input layer
        inputLayerSize = None

        layersBuffer = []

        # loop through logfile
        for line in logfile:

            # read input layer size
            if line[0:19] == "+++input_layer_size":
                inputLayerSize = int(line.split("===>")[1])

            # if a buffer is empty, fill it
            if curLayerLabel is None or\
                curLayerWeights is None or\
                curLayerBias is None or\
                curLayerSize is None:


                # is current the line a new layer?
                if line[0] == "#":
                    curLayerLabel = line.replace("###", "").split("===>")[1]

                # is current the line representing the weights of a layer?
                if line[0] == "w":
                    weightStr = line.split("===>")[1]
                    curLayerWeights = self.__decodeNumpyArray(weightStr)

                # is current the line representing the bias of a layer?
                if line[0] == "b":
                    biasStr = line.split("===>")[1]
                    curLayerBias = self.__decodeNumpyArray(biasStr)

                if line[0] == "s":
                    curLayerSize = int(line.split("===>")[1])

            # buffers full, save one layers data and erase buffers
            if curLayerLabel is not None and\
                curLayerWeights is not None and\
                curLayerBias is not None and\
                curLayerSize is not None:

                layersBuffer.append(
                    {"label": curLayerLabel,
                     "weights": curLayerWeights,
                     "bias": curLayerBias,
                     "size": curLayerSize}
                )
                curLayerLabel = None
                curLayerWeights = None
                curLayerBias = None
                curLayerSize= None


        # loop through layersBuffer and create the layers
        layerHolder = []

        for key, bufferedLayer in enumerate(layersBuffer):

            if key == 0:
                parentLayerSize = inputLayerSize
            else:
                parentLayerSize = layersBuffer[key-1]["size"]

            newLayer = LayerClass.Layer(bufferedLayer["size"], parentLayerSize, bufferedLayer["label"])
            newLayer.setWeights(bufferedLayer["weights"])
            newLayer.setBias(bufferedLayer["bias"])

            layerHolder.append(newLayer)

        # set the new layers for the net
        self.hiddenAndOutputLayer = layerHolder
        self.hiddenLayer = layerHolder[0:-1]
        self.outputLayer = layerHolder[-1]


    def __checkForErrorsInit(self, in_input, hiddenLayerList, outputLayerLength):
        """
        Checks for errors before starting current init things

        :param in_input: input layers data
        :param hiddenLayerList: hidden layer configuration
        :param outputLayerLength: length of the output layer
        """

        errStr = "!!! ERROR !!!\n"
        errStr += "\n"

        # check if input data and expected output and hiddenLayerList are lists
        if not ((type(in_input) is int) and (type(hiddenLayerList) is list) and (len(hiddenLayerList) > 0)):
            errStr += "Error: in_input has to be an integer, hiddenLayerList has to be a list of integers"
            sys.exit(errStr)

        # check output layer length / size
        if not ((type(outputLayerLength) is int) and (outputLayerLength > 0)):
            errStr += "Error: outputLayerLength has to be an integer > 0"
            sys.exit(errStr)


    def __checkForErrorsTeach(self, inputData, expectedOutputData):
        """
        Checks for error before starting to teach the net

        :param inputData: input data for input layer
        :param expectedOutputData: expected output data for the neural net
        """

        errStr = "!!! ERROR !!!\n"
        errStr += "\n"

        # check if inputData and expectedOutputData are lists
        if not ((type(inputData) is list) and (type(expectedOutputData) is list)):
            errStr += "Error: inputData and expectedOutputData have to be lists"
            sys.exit(errStr)

        # check if inputData and expectedOutputData are empty
        if not ((len(inputData) > 0) and (len(expectedOutputData) > 0)):
            errStr += "Error: inputData and expectedOutputData must not be empty"
            sys.exit(errStr)

        # check if inputData and expectedOutputData have the same length
        if not (len(inputData) == len(expectedOutputData) > 0):
            errStr += "Error: inputData and expectedOutputData have to be of the same length"
            sys.exit(errStr)
