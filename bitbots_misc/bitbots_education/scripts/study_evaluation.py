from scipy import stats
import numpy as np
import pandas as pd


data = pd.read_csv("robot_quiz_responses.csv")
quiz_data = None
def calculate_gender():
    women = data[data['Zu welchem Geschlecht fühlst du dich zugehörig?']=='weiblich']
    men = data[data['Zu welchem Geschlecht fühlst du dich zugehörig?']=='männlich']
    divers = data[data['Zu welchem Geschlecht fühlst du dich zugehörig?']=='divers']
    total_count_women = women['Zu welchem Geschlecht fühlst du dich zugehörig?'].count()
    total_count_men = men['Zu welchem Geschlecht fühlst du dich zugehörig?'].count()
    total_count_divers = divers['Zu welchem Geschlecht fühlst du dich zugehörig?'].count()
    print ("Weiblich:", total_count_women, " Männlich:", total_count_men, " Divers:",total_count_divers)

def calculate_quiz_score(row):
    score = 0
    if (row['Wo befindet sich der Akku im Roboter der Hamburg Bit-Bots?'] == 'Im unteren Teil des Torsos'):
        score += 1
    if (row['Was ist kein Teil des Verhaltens der Hamburg Bit-Bots?'] == 'Reflexe'):
        score += 1
    if (row['Wofür benutzt ein Fußballspielender Roboter seine Motoren nicht?'] == 'Fallerkennung'):
        score += 1
    if (row['Welche Bildelemente erkennt die Bilderkennung der Hamburg Bit-Bots?'] == 'Ball, Roboter, Linien'):
        score += 1
    if (row['Wobei wird die innertiale Messeinheit (IMU) nicht gebraucht?'] == 'Bilderkennung'):
        score += 1
    if (row['Welcher Vergleich zum Menschen ist nicht richtig?'] == 'Die Bilderkennung gleicht dem Tastsinn'):
        score += 1
    if (row['Warum bewegt der Roboter seinen Kopf?'] == 'um noch mehr vom Feld sehen zu können'):
        score += 1
    if (row['Müssen Roboter in der humanoid kid size league des Robocups aufstehen können?'] == 'ja, sie müssen im Spiel selbstständig aufstehen können um spielen zu dürfen'):
        score += 1   

    return score
 

def add_quiz_score():
    
    scores = []
    for index, row in data.iterrows():
        scores.append(calculate_quiz_score(data.iloc[index]))

    quiz_score = pd.Series(scores, name='Quiz Score')
    quiz_data = pd.concat([data, quiz_score], axis=1)

    print(quiz_data['Quiz Score'])


calculate_gender()
add_quiz_score()
