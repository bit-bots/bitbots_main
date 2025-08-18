import pandas as pd
import quiz_score as qs
import sus_score as ss
import matplotlib.pyplot as plt
import numpy as np

class StudyEvaluation:
    def __init__(self, study_data1, study_data2, quiz_data):

        # Assert that the ID columns are the same in both study datasets
        assert study_data1["Demographic00. Gib deine ID ei.. "].equals(study_data2["Demographic00. Gib deine ID ei.. "]), "ID columns in study datasets do not match."

        self.study_score1 = qs.QuizScore(study_data1, "study1")
        self.study_data1 = self.study_score1.quiz_data
        self.sus1 = ss.SUSScore(self.study_data1, "study1")

        self.study_score2 = qs.QuizScore(study_data2, "study2")
        self.study_data2 = self.study_score2.quiz_data
        self.sus2 = ss.SUSScore(self.study_data2, "study2")

        self.quiz_score = qs.QuizScore(quiz_data, "quiz")
        self.quiz_data = self.quiz_score.quiz_data




if __name__ == "__main__":
    data_raw = pd.read_csv("quizAnswerOptions.csv", delimiter=";")

    test_data1 = pd.read_csv("ersterDurchganag.csv", delimiter=";")

    test_data2 = pd.read_csv("test_zweiterDurchganag.csv", delimiter=";")

    eval = StudyEvaluation(test_data1, test_data2, data_raw)
    print(eval.quiz_data["Quiz Score"])

    # This code block is used to visualize the quiz scores using a box plot.
    fig, axs = plt.subplots(figsize=(4,4))
    eval.quiz_data['Quiz Score'].plot.box(ax=axs)
    axs.set_ylabel("Quiz Score")
    axs.set_yticks(range(0, 16, 1))
    plt.show()
    eval.quiz_data[['Quiz Score']].plot.box()
    plt.show()
