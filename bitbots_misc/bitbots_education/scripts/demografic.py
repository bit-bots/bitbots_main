import pandas as pd


class Demographic:
    def __init__(self, data, column_name):
        
        self.data = data
        self.column_name = column_name
        self.demographic_data = None
        self.evaluateDemographics()

    def evaluateDemographics(self):
        demographic_list = []
        for row in self.data:
            demographic = self.getDemographicFromRow(row)
            demographic_list.append(demographic)

        self.demographic_data = pd.Series(demographic_list, name="Demographic Score")

    def getDemographicFromRow(self, row):
        demographic = 0
        print(row)
        match row:
            case "A1":
                demographic = 1
            case "A2":
                demographic = 2
            case "A3":
                demographic = 3
            case "A4":
                demographic = 4
            case "A5":
                demographic = 5
            case "A6":
                demographic = 6
            case "A7":
                demographic = 7

        if (demographic == 0):
            match row:
                case "AO01":
                    demographic = 1
                case "AO02":
                    demographic = 2
                case "AO03":
                    demographic = 3
                case "AO04":
                    demographic = 4
                case "AO05":
                    demographic = 5
                case "AO06":
                    demographic = 6
                case "AO07":
                    demographic = 7

        return demographic
