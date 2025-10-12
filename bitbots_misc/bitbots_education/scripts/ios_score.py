import pandas as pd


class IOSScore:
    def __init__(self, data, questionnaire):
        ios_study1_list = [
            "G05Q35. Welche der Abbi.. ",
            "q0r825. Welche der Abbi.. ",
        ]

        ios_study2_list = [
            "r760q0. Welche der Abbi.. ",
            "r783q0. Welche der Abbi.. "
        ]
        self.data = data
        self.ios_data = None
        if questionnaire == "study1":
            self.evaluateIOS(ios_study1_list)
        elif questionnaire == "study2":
            self.evaluateIOS(ios_study2_list)

    def evaluateIOS(self, ios_list):
        ios_robot_list = []
        ios_group_list = []
        for _, row in self.data.iterrows():
            ios_robot, ios_group = self.getIOSScoreFromRow(row, ios_list)
            ios_robot_list.append(ios_robot)
            ios_group_list.append(ios_group)

        ios_robot_score = pd.Series(ios_robot_list, name="IOS Robot Score")
        ios_group_score = pd.Series(ios_group_list, name="IOS Group Score")
        self.ios_data = pd.concat([self.data, ios_robot_score, ios_group_score], axis=1)

    def getIOSScoreFromRow(self, row, ios_list):
        ios_robot = 0
        ios_group = 0
        match row[ios_list[0]]:
            case "AO01":
                ios_robot = 1
            case "AO02":
                ios_robot = 2
            case "AO03":
                ios_robot = 3
            case "AO04":
                ios_robot = 4
            case "AO05":
                ios_robot = 5
            case "AO06":
                ios_robot = 6
            case "AO07":
                ios_robot = 7

        match row[ios_list[1]]:
            case "AO01":
                ios_group = 1
            case "AO02":
                ios_group = 2
            case "AO03":
                ios_group = 3
            case "AO04":
                ios_group = 4
            case "AO05":
                ios_group = 5
            case "AO06":
                ios_group = 6
            case "AO07":
                ios_group = 7

        return ios_robot, ios_group
