import pandas as pd

data_raw = pd.read_csv("robot_quiz_responses.csv")
data = data_raw.rename(
    columns={
        "Gib deinen Studiengang oder Beruf an.": "Studiengang/Beruf",
        "Zu welchem Geschlecht fühlst du dich zugehörig?": "Geschlecht",
        "Wie alt bist du?": "Alter",
        "Wie viel Kontakt hattest du zu humanoiden Robotern?": "Vorwissen",
        "Antwort ID": "ID",
    }
)
quiz_data = None


def calculate_gender():
    women = data[data["Geschlecht"] == "weiblich"]
    men = data[data["Geschlecht"] == "männlich"]
    divers = data[data["Geschlecht"] == "divers"]
    total_count_women = women["Geschlecht"].count()
    total_count_men = men["Geschlecht"].count()
    total_count_divers = divers["Geschlecht"].count()
    return total_count_divers, total_count_men, total_count_women


# A Naive recursive implementation of LCS problem


# Returns length of LCS for s1[0..m-1], s2[0..n-1]
def lcsRec(s1, s2, m, n):
    # Base case: If either string is empty, the length of LCS is 0
    if m == 0 or n == 0:
        return 0

    # If the last characters of both substrings match
    if s1[m - 1] == s2[n - 1]:
        # Include this character in LCS and recur for remaining substrings
        return 1 + lcsRec(s1, s2, m - 1, n - 1)

    else:
        # If the last characters do not match
        # Recur for two cases:
        # 1. Exclude the last character of S1
        # 2. Exclude the last character of S2
        # Take the maximum of these two recursive calls
        return max(lcsRec(s1, s2, m, n - 1), lcsRec(s1, s2, m - 1, n))


def lcs(s1, s2):
    m = len(s1)
    n = len(s2)
    return lcsRec(s1, s2, m, n)


def make_sequence_string(row):
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


def calculate_quiz_score(row):
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

    motor_string, behavior_string = make_sequence_string(row)
    score += lcs(motor_string, "123")  # Expected motor sequence
    score += lcs(behavior_string, "1234")
    return score


def add_quiz_score():
    scores = []
    for index, row in data.iterrows():
        scores.append(calculate_quiz_score(data.iloc[index]))

    quiz_score = pd.Series(scores, name="Quiz Score")
    quiz_data = pd.concat([data, quiz_score], axis=1)

    return quiz_data


count_divers, count_men, count_women = calculate_gender()
quiz_data = add_quiz_score()


""" This code block is used to visualize the quiz scores using a box plot.
fig, axs = plt.subplots(figsize=(4,4))
quiz_data['Quiz Score'].plot.box(ax=axs)
axs.set_ylabel("Quiz Score")
axs.set_yticks(np.arange(0, 16, 1)), ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15']
plt.show()
quiz_data[['Quiz Score']].plot.box()
plt.show()
"""
