__author__ = 'daniel'


# imports
import NeuralNetworkClass as NNC


# load data
file = open("tamara_cali_u2000.log")
file2000 = file.read().split("\n")
file.close()

key = 0
while key < len(file2000):
    file2000[key] = file2000[key].replace("nan", "0.0")
    file2000[key] = file2000[key].strip().split(" ")
    key += 1

def createDict(listdata):

    container = []
    for elem in listdata:
        container.append({
            "id": elem[0],
            "center_u_raw": elem[5],
            "center_v_raw": elem[6],
            "center_u_filtered": elem[11],
            "center_v_filtered": elem[12]
        })
    return container

dataU2000V0 = createDict(file2000[230:331])
dataU2000VMinus500 = createDict(file2000[650:751])
dataU2000VPlus500 = createDict(file2000[1650:1751])
dataU2000VMinus1000 = createDict(file2000[2050:2151])
dataU2000VPlus1000 = createDict(file2000[2250:2351])



# data and net config
inputLength = 2

inputData = []
inputData.extend([
    [float(item["center_u_filtered"])/10000.0, float(item["center_v_filtered"])/10000.0] for item in dataU2000V0
])
inputData.extend([
    [float(item["center_u_filtered"])/10000.0, float(item["center_v_filtered"])/10000.0] for item in dataU2000VMinus500
])
inputData.extend([
    [float(item["center_u_filtered"])/10000.0, float(item["center_v_filtered"])/10000.0] for item in dataU2000VPlus500
])
inputData.extend([
    [float(item["center_u_filtered"])/10000.0, float(item["center_v_filtered"])/10000.0] for item in dataU2000VMinus1000
])
inputData.extend([
    [float(item["center_u_filtered"])/10000.0, float(item["center_v_filtered"])/10000.0] for item in dataU2000VPlus1000
])

hidden = [10, 5]

outputLength = 2

expectedOutput = []

expectedOutput.extend([2000.0/10000.0, 0.0] for i in range(0, 101))
expectedOutput.extend([2000.0/10000.0, -500.0/10000.0] for i in range(0, 101))
expectedOutput.extend([2000.0/10000.0, 500.0/10000.0] for i in range(0, 101))
expectedOutput.extend([2000.0/10000.0, -1000.0/10000.0] for i in range(0, 101))
expectedOutput.extend([2000.0/10000.0, 1000.0/10000.0] for i in range(0, 101))


# create the net
net = NNC.NeuralNetwork(inputLength, hidden, outputLength)

# teach the net
net.teach(inputData, expectedOutput, 10000, 0.2)
net.saveWeightsAndBias()

# load net data
#net.loadWeightsAndBias()
#net.printNetWeights()

print(net.calculate([2000.0/10000.0, 0.0]))