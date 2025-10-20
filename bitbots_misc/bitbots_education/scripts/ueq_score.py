import pandas as pd


class UEQScore:
    def __init__(self, data, questionnaire):
        ueq_study1_list = [
            "r446q0[UEQ1]. User Experience..  [behindernd|unte.. ]",
            "r446q0[UEQ2]. User Experience..  [kompliziert|ein.. ]",
            "r446q0[UEQ3]. User Experience..  [ineffizient|eff.. ]",
            "r446q0[UEQ4]. User Experience..  [verwirrend|über.. ]",
            "r446q0[UEQ5]. User Experience..  [langweilig|span.. ]",
            "r446q0[UEQ6]. User Experience..  [uninteressant|i.. ]",
            "r446q0[UEQ7]. User Experience..  [konventionell|o.. ]",
            "r446q0[UEQ8]. User Experience..  [herkömmlich|neu.. ]"
        ]

        ueq_study2_list = [
            "G04Q14[UEQ1]. User Experience..  [behindernd|unte.. ]",
            "G04Q14[UEQ2]. User Experience..  [kompliziert|ein.. ]",
            "G04Q14[UEQ3]. User Experience..  [ineffizient|eff.. ]",
            "G04Q14[UEQ4]. User Experience..  [verwirrend|über.. ]",
            "G04Q14[UEQ5]. User Experience..  [langweilig|span.. ]",
            "G04Q14[UEQ6]. User Experience..  [uninteressant|i.. ]",
            "G04Q14[UEQ7]. User Experience..  [konventionell|o.. ]",
            "G04Q14[UEQ8]. User Experience..  [herkömmlich|neu.. ]"

        ]
        self.data = data
        self.ueq_data = None
        if questionnaire == "study1":
            self.evaluateUEQ(ueq_study1_list)
        elif questionnaire == "study2":
            self.evaluateUEQ(ueq_study2_list)

    def evaluateUEQ(self, ueq_list):
        pragmatic_list = []
        hedonic_list = []
        overall_list = []
        for _, row in self.data.iterrows():
            pragmatic, hedonic, overall = self.getUEQScoreFromRow(row, ueq_list)
            pragmatic_list.append(pragmatic)
            hedonic_list.append(hedonic)
            overall_list.append(overall)

        ueq_pragmatic_score = pd.Series(pragmatic_list, name="UEQs Pragmatic Score")
        ueq_hedonic_score = pd.Series(hedonic_list, name="UEQs Hedonic Score")
        ueq_overall_score = pd.Series(overall_list, name="UEQs Overall Score")
        self.ueq_data = pd.concat([self.data, ueq_pragmatic_score, ueq_hedonic_score, ueq_overall_score], axis=1)

    def getUEQScoreFromRow(self, row, ueq_list):
        pragmatic = 0
        hedonic = 0
        overall = 0
        for i in range(0, 4):
            match row[ueq_list[i]]:
                case "AO01":
                    pragmatic += 1
                case "AO02":
                    pragmatic += 2
                case "AO03":
                    pragmatic += 3
                case "AO04":
                    pragmatic += 4
                case "AO05":
                    pragmatic += 5
                case "AO06":
                    pragmatic += 6
                case "AO07":
                    pragmatic += 7
        
        for i in range(4, 8):
            match row[ueq_list[i]]:
                case "AO01":
                    hedonic += 1
                case "AO02":
                    hedonic += 2
                case "AO03":
                    hedonic += 3
                case "AO04":
                    hedonic += 4
                case "AO05":
                    hedonic += 5
                case "AO06":
                    hedonic += 6
                case "AO07":
                    hedonic += 7
        
        overall = pragmatic + hedonic


        return pragmatic/4, hedonic/4, overall/8
