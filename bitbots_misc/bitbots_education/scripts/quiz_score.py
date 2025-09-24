import pandas as pd


class QuizScore:
    def __init__(self, data, questionnaire):
        self.quiz_list = [
            "Wo befindet sic.. ",
            "Was ist kein Te.. ",
            "Wofür benutzt e.. ",
            "Welche Bildelem.. ",
            "Wobei wird die .. ",
            "Welcher Verglei.. ",
            "Warum bewegt de.. ",
            "Müssen Roboter .. ",
        ]
        self.study1_list = [
            "G01Q01. Wo befindet sic.. ",
            "G01Q10. Was ist kein Te.. ",
            "G01Q03. Wofür benutzt e.. ",
            "G01Q04. Welche Bildelem.. ",
            "G01Q06. Wobei wird die .. ",
            "G01Q07. Welcher Verglei.. ",
            "G01Q08. Warum bewegt de.. ",
            "G01Q09. Müssen Roboter .. ",
        ]

        self.study2_list = [
            "r335q0. Wo befindet sic.. ",
            "r668q0. Was ist kein Te.. ",
            "r798q0. Wofür benutzt e.. ",
            "r498q0. Welche Bildelem.. ",
            "r736q0. Wobei wird die .. ",
            "r119q0. Welcher Verglei.. ",
            "r89q0. Warum bewegt de.. ",
            "r1q0. Müssen Roboter .. ",
        ]

        self.rank_motor_quiz_list = [
            "Ordne Motoren, ..  [Rank 1]",
            "Ordne Motoren, ..  [Rank 2]",
            "Ordne Motoren, ..  [Rank 3]",
        ]

        self.rank_behavior_quiz_list = [
            "Ordne die Aktio..  [Rank 1]",
            "Ordne die Aktio..  [Rank 2]",
            "Ordne die Aktio..  [Rank 3]",
            "Ordne die Aktio..  [Rank 4]",
        ]

        self.rank_motor_study1_list = [
            "G01Q02[1]. Ordne Motoren, ..  [Rank 1]",
            "G01Q02[2]. Ordne Motoren, ..  [Rank 2]",
            "G01Q02[3]. Ordne Motoren, ..  [Rank 3]",
        ]

        self.rank_behavior_study1_list = [
            "G01Q05[1]. Ordne die Aktio..  [Rank 1]",
            "G01Q05[2]. Ordne die Aktio..  [Rank 2]",
            "G01Q05[3]. Ordne die Aktio..  [Rank 3]",
            "G01Q05[4]. Ordne die Aktio..  [Rank 4]",
        ]

        self.rank_motor_study2_list = [
            "r427q0[1]. Ordne Motoren, ..  [Rank 1]",
            "r427q0[2]. Ordne Motoren, ..  [Rank 2]",
            "r427q0[3]. Ordne Motoren, ..  [Rank 3]",
        ]

        self.rank_behavior_study2_list = [
            "r81q0[1]. Ordne die Aktio..  [Rank 1]",
            "r81q0[2]. Ordne die Aktio..  [Rank 2]",
            "r81q0[3]. Ordne die Aktio..  [Rank 3]",
            "r81q0[4]. Ordne die Aktio..  [Rank 4]",
        ]

        self.data = data
        self.quiz_data = None

        if questionnaire == "study1":
            self.add_quiz_score(self.study1_list, self.rank_motor_study1_list, self.rank_behavior_study1_list)
        elif questionnaire == "quiz":
            self.add_quiz_score(self.quiz_list, self.rank_motor_quiz_list, self.rank_behavior_quiz_list)
        elif questionnaire == "study2":
            self.add_quiz_score(self.study2_list, self.rank_motor_study2_list, self.rank_behavior_study2_list)

    def add_quiz_score(self, column_list, rank_motor_list, rank_behavior_list):
        scores = []
        for _, row in self.data.iterrows():
            scores.append(self.calculate_quiz_score(row, column_list, rank_motor_list, rank_behavior_list))

        quiz_score = pd.Series(scores, name="Quiz Score")
        self.quiz_data = pd.concat([self.data, quiz_score], axis=1)

    def calculate_quiz_score(self, row, column_list, rank_motor_list, rank_behavior_list):
        score = 0
        if row[column_list[0]] == "AO01":
            score += 1
        if row[column_list[1]] == "AO04":
            score += 1
        if row[column_list[2]] == "AO04":
            score += 1
        if row[column_list[3]] == "AO01":
            score += 1
        if row[column_list[4]] == "AO04":
            score += 1
        if row[column_list[5]] == "AO02":
            score += 1
        if row[column_list[6]] == "AO02":
            score += 1
        if row[column_list[7]] == "AO01":
            score += 1

        motor_string, behavior_string = self.make_sequence_string(row, rank_motor_list, rank_behavior_list)
        score += self.lcs(motor_string, "123")  # Expected motor sequence
        score += self.lcs(behavior_string, "1234")
        return score

    def make_sequence_string(self, row, rank_motor_list, rank_behavior_list):
        rankMotors1 = row[rank_motor_list[0]]
        rankMotors2 = row[rank_motor_list[1]]
        rankMotors3 = row[rank_motor_list[2]]

        rankBehavior1 = row[rank_behavior_list[0]]
        rankBehavior2 = row[rank_behavior_list[1]]
        rankBehavior3 = row[rank_behavior_list[2]]
        rankBehavior4 = row[rank_behavior_list[3]]

        rankListMotor = [rankMotors1, rankMotors2, rankMotors3]
        rankListBehavior = [rankBehavior1, rankBehavior2, rankBehavior3, rankBehavior4]
        motor_sequence = []
        behavoir_sequence = []
        for i in rankListMotor:
            match i:
                case "AO01":
                    motor_sequence.append("1")
                case "AO02":
                    motor_sequence.append("2")
                case "AO03":
                    motor_sequence.append("3")

        for i in rankListBehavior:
            match i:
                case "AO01":
                    behavoir_sequence.append("1")
                case "AO02":
                    behavoir_sequence.append("2")
                case "AO03":
                    behavoir_sequence.append("3")
                case "AO04":
                    behavoir_sequence.append("4")

        delimiter = ""
        motor_string = delimiter.join(motor_sequence)
        behavoir_string = delimiter.join(behavoir_sequence)
        return motor_string, behavoir_string

    # A Naive recursive implementation of LCS problem

    # Returns length of LCS for s1[0..m-1], s2[0..n-1]
    def lcsRec(self, s1, s2, m, n):
        # Base case: If either string is empty, the length of LCS is 0
        if m == 0 or n == 0:
            return 0

        # If the last characters of both substrings match
        if s1[m - 1] == s2[n - 1]:
            # Include this character in LCS and recur for remaining substrings
            return 1 + self.lcsRec(s1, s2, m - 1, n - 1)

        else:
            # If the last characters do not match
            # Recur for two cases:
            # 1. Exclude the last character of S1
            # 2. Exclude the last character of S2
            # Take the maximum of these two recursive calls
            return max(self.lcsRec(s1, s2, m, n - 1), self.lcsRec(s1, s2, m - 1, n))

    def lcs(self, s1, s2):
        m = len(s1)
        n = len(s2)
        return self.lcsRec(s1, s2, m, n)
