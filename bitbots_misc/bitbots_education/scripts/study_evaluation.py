import pandas as pd


class QuizEvaluation:
    def __init__(self, data):
        self.data = data
        self.quiz_data = None
        self.add_quiz_score()
            
    def calculate_gender(self):
        women = self.data[self.data["Geschlecht"] == "weiblich"]
        men = self.data[self.data["Geschlecht"] == "männlich"]
        divers = self.data[self.data["Geschlecht"] == "divers"]
        total_count_women = women["Geschlecht"].count()
        total_count_men = men["Geschlecht"].count()
        total_count_divers = divers["Geschlecht"].count()
        return total_count_divers, total_count_men, total_count_women

    def add_quiz_score(self):
        scores = []
        for index, row in self.data.iterrows():
            scores.append(self.calculate_quiz_score(self.data.iloc[index]))

        quiz_score = pd.Series(scores, name="Quiz Score")
        self.quiz_data = pd.concat([self.data, quiz_score], axis=1)

    def calculate_quiz_score(self, row):
        score = 0
        if row["Wo befindet sich der Akku im Roboter der Hamburg Bit-Bots?"] == "Im unteren Teil des Torsos":
            score += 1
        if row["Was ist kein Teil des Verhaltens der Hamburg Bit-Bots?"] == "Reflexe":
            score += 1
        if row["Wofür benutzt ein Fußballspielender Roboter seine Motoren nicht?"] == "Fallerkennung":
            score += 1
        if row["Welche Bildelemente erkennt die Bilderkennung der Hamburg Bit-Bots?"] == "Ball, Roboter, Linien":
            score += 1
        if row["Wobei wird die innertiale Messeinheit (IMU) nicht gebraucht?"] == "Bilderkennung":
            score += 1
        if row["Welcher Vergleich zum Menschen ist nicht richtig?"] == "Die Bilderkennung gleicht dem Tastsinn":
            score += 1
        if row["Warum bewegt der Roboter seinen Kopf?"] == "um noch mehr vom Feld sehen zu können":
            score += 1
        if (
            row["Müssen Roboter in der humanoid kid size league des Robocups aufstehen können?"]
            == "ja, sie müssen im Spiel selbstständig aufstehen können um spielen zu dürfen"
        ):
            score += 1

        motor_string, behavior_string = self.make_sequence_string(row)
        score += self.lcs(motor_string, "123")  # Expected motor sequence
        score += self.lcs(behavior_string, "1234")
        return score
    def make_sequence_string(self, row):
        rankMotors1 = row[
            "Ordne Motoren, die die längste Zeit innerhalb eines Fußballspieles bewegt werden, nach oben. [Rank 1]"
        ]
        rankMotors2 = row[
            "Ordne Motoren, die die längste Zeit innerhalb eines Fußballspieles bewegt werden, nach oben. [Rank 2]"
        ]
        rankMotors3 = row[
            "Ordne Motoren, die die längste Zeit innerhalb eines Fußballspieles bewegt werden, nach oben. [Rank 3]"
        ]

        rankBehavior1 = row["Ordne die Aktionen nach oben die zuerst ausgeführt werden um einen Ball zu kicken. [Rank 1]"]
        rankBehavior2 = row["Ordne die Aktionen nach oben die zuerst ausgeführt werden um einen Ball zu kicken. [Rank 2]"]
        rankBehavior3 = row["Ordne die Aktionen nach oben die zuerst ausgeführt werden um einen Ball zu kicken. [Rank 3]"]
        rankBehavior4 = row["Ordne die Aktionen nach oben die zuerst ausgeführt werden um einen Ball zu kicken. [Rank 4]"]

        rankList = [rankMotors1, rankMotors2, rankMotors3, rankBehavior1, rankBehavior2, rankBehavior3, rankBehavior4]
        motor_sequence = []
        behavoir_sequence = []
        for i in rankList:
            match i:
                case "Motor im Hals":
                    motor_sequence.append("1")
                case "Motor in der Hüfte":
                    motor_sequence.append("2")
                case "Motor in der Schulter":
                    motor_sequence.append("3")
                case "Ball finden":
                    behavoir_sequence.append("1")
                case "Zum Ball gehen":
                    behavoir_sequence.append("2")
                case "Auf der Stelle gehen":
                    behavoir_sequence.append("3")
                case "Kick ausführen":
                    behavoir_sequence.append("4")

        delimiter = ","
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


class StudyEvaluation:
    def __init__(self, study_data, quiz_data):
        self.data = study_data
        self.quiz_evaluation = QuizEvaluation(quiz_data)
        self.quiz_data = self.quiz_evaluation.quiz_data

    def evaluateSUS(self):
        sus_score = 0
        for _, row in self.data.iterrows():
            sus_score = self.getSUSScoreFromRow(row)

    def getSUSScoreFromRow(self, row):
        print (row)
        
        SUSlist = [
            row['SUS 1'],
            row['SUS 2'],
            row['SUS 3'],
            row['SUS 4'],
            row["SUS 5"],
            row["SUS 6"],
            row["SUS 7"],
            row["SUS 8"],
            row["SUS 9"],
            row["SUS 10"],
        ]

        for i in range(0, len(SUSlist), 1):
            print("SUSlist", SUSlist[i], i)



""" This code block is used to visualize the quiz scores using a box plot.
fig, axs = plt.subplots(figsize=(4,4))
quiz_data['Quiz Score'].plot.box(ax=axs)
axs.set_ylabel("Quiz Score")
axs.set_yticks(np.arange(0, 16, 1)), ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15']
plt.show()
quiz_data[['Quiz Score']].plot.box()
plt.show()
"""


data_raw = pd.read_csv("robot_quiz_responses.csv")
quiz_csv_data = data_raw.rename(
    columns={
        "Gib deinen Studiengang oder Beruf an.": "Studiengang/Beruf",
        "Zu welchem Geschlecht fühlst du dich zugehörig?": "Geschlecht",
        "Wie alt bist du?": "Alter",
        "Wie viel Kontakt hattest du zu humanoiden Robotern?": "Vorwissen",
        "Antwort ID": "ID",
    }
)

test_data = pd.read_csv("/homes/21wedmann/Downloads/results-survey667957(1).csv", delimiter=";")

test_renamed = test_data.rename(
    columns={
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich denke, dass ich dieses Produkt häufig verwenden möchte]": "SUS 1",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich fand das Produkt unnötig komplex]": "SUS 2",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich dachte, dass Produkt war einfach zu bedienen]": "SUS 3",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich denke, dass ich die Unterstützung einer technischen Person brauche, um dieses Produkt nutzen zu können  ]": "SUS 4",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich fand, die verschiedenen Funktionen in diesem Produkt waren gut integriert]": "SUS 5",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich dachte, dass dieses produkt nicht konsisitent genug war]": "SUS 6",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich würde mir vorstellen, dass die meisten Leute sehr schnell lernen würden, dieses Produkt zu benutzen]": "SUS 7",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich fand dieses System sehr umständlich zu benutzen]": "SUS 8",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich habe mich sehr selbstsicher gefühlt, dieses Produkt zu verwenden]": "SUS 9",
        "System Usability Scale  Bitte denken Sie nicht zu lange über jede Aussage nach. Die Aussagen sind bezüglich der jeweiligen Kondition also Audio oder Web Apllikation zu beantworten.  [Ich musste eine Menge Dinge lernen, bevor ich mit diesem Produkt loslegen konnte]": "SUS 10",
    })
eval = StudyEvaluation(test_renamed, quiz_csv_data)
eval.evaluateSUS()
