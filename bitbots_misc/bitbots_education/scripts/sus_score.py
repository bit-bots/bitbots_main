import pandas as pd


class SUSScore:
    def __init__(self, data, questionnaire):
        sus_study1_list = [
            "r418q0[SQ001]. System Usabilit..  [Ich denke, dass.. ]",
            "r418q0[SQ002]. System Usabilit..  [Ich fand das Pr.. ]",
            "r418q0[SQ003]. System Usabilit..  [Ich dachte, das.. ]",
            "r418q0[SQ004]. System Usabilit..  [Ich denke, dass.. ]",
            "r418q0[SQ005]. System Usabilit..  [Ich fand, die v.. ]",
            "r418q0[SQ006]. System Usabilit..  [Ich dachte, das.. ]",
            "r418q0[SQ007]. System Usabilit..  [Ich würde mir v.. ]",
            "r418q0[SQ008]. System Usabilit..  [Ich fand dieses.. ]",
            "r418q0[SQ009]. System Usabilit..  [Ich habe mich s.. ]",
            "r418q0[SQ010]. System Usabilit..  [Ich musste eine.. ]",
        ]

        sus_study2_list = [
            "G04Q13[SQ001]. System Usabilit..  [Ich denke, dass.. ]",
            "G04Q13[SQ002]. System Usabilit..  [Ich fand das Pr.. ]",
            "G04Q13[SQ003]. System Usabilit..  [Ich dachte, das.. ]",
            "G04Q13[SQ004]. System Usabilit..  [Ich denke, dass.. ]",
            "G04Q13[SQ005]. System Usabilit..  [ich fand, die v.. ]",
            "G04Q13[SQ006]. System Usabilit..  [Ich dachte, das.. ]",
            "G04Q13[SQ007]. System Usabilit..  [Ich würde mir v.. ]",
            "G04Q13[SQ008]. System Usabilit..  [Ich fand dieses.. ]",
            "G04Q13[SQ009]. System Usabilit..  [Ich habe mich s.. ]",
            "G04Q13[SQ010]. System Usabilit..  [Ich musste eine.. ]",
        ]
        self.data = data
        self.sus_score = 0
        self.sus_data = None
        if questionnaire == "study1":
            self.evaluateSUS(sus_study1_list)
        elif questionnaire == "study2":
            self.evaluateSUS(sus_study2_list)

    def evaluateSUS(self, sus_list):
        sus_scores = []
        for _, row in self.data.iterrows():
            sus_scores.append(self.getSUSScoreFromRow(row, sus_list))

        sus_score = pd.Series(sus_scores, name="SUS Score")
        self.sus_data = pd.concat([self.data, sus_score], axis=1)

    def getSUSScoreFromRow(self, row, sus_list):
        SUSlist = [
            row[sus_list[0]],
            row[sus_list[1]],
            row[sus_list[2]],
            row[sus_list[3]],
            row[sus_list[4]],
            row[sus_list[5]],
            row[sus_list[6]],
            row[sus_list[7]],
            row[sus_list[8]],
            row[sus_list[9]],
        ]
        sus_score = 0
        for i in range(0, len(SUSlist), 2):
            match SUSlist[i]:
                case "AO01":
                    sus_score += 0
                case "AO02":
                    sus_score += 1
                case "AO03":
                    sus_score += 2
                case "AO04":
                    sus_score += 3
                case "AO05":
                    sus_score += 4

        for i in range(1, len(SUSlist), 2):
            match SUSlist[i]:
                case "AO01":
                    sus_score += 4
                case "AO02":
                    sus_score += 3
                case "AO03":
                    sus_score += 2
                case "AO04":
                    sus_score += 1
                case "AO05":
                    sus_score += 0

        sus_score = sus_score * 2.5  # Convert to a score out of 100
        return sus_score
