import pandas as pd
import quiz_score as qs
import sus_score as ss
import log_evaluation as le
import os 


class StudyEvaluation:
    def __init__(self, study_data1, study_data2, quiz_data, logs_list):
        # Assert that the ID columns are the same in both study datasets
        assert study_data1["Demographic00. Gib deine ID ei.. "].equals(
            study_data2["Demographic00. Gib deine ID ei.. "]
        ), "ID columns in study datasets do not match."

        # Evaluate logs
        log_data_list = []
        for log_file in logs_list:
            log_data = pd.read_csv(os.path.join("/homes/21wedmann/colcon_ws/src/bitbots_main/bitbots_misc/bitbots_education/scripts/logs", log_file))
            log_data_sorted = log_data.sort_values(by=["VP", "Timestamp"])
            log_data_sorted["Page"].fillna("dashboard", inplace=True)
            log_data_list.append(log_data_sorted)

        self.log_eval = le.LogEvaluation(log_data_list)

        study_score1 = qs.QuizScore(study_data1, "study1")
        study_data1 = study_score1.quiz_data
        sus1 = ss.SUSScore(study_data1, "study1")
        self.full_data1 = sus1.sus_data

        study_score2 = qs.QuizScore(study_data2, "study2")
        study_data2 = study_score2.quiz_data
        sus2 = ss.SUSScore(study_data2, "study2")
        self.full_data2 = sus2.sus_data

        quiz_score = qs.QuizScore(quiz_data, "quiz")
        self.quiz_data = quiz_score.quiz_data

    def evaluate_quiz_scores(self):
        print(self.quiz_data["Quiz Score"].mean())
        print(self.full_data1["Quiz Score"].mean())
        print(self.full_data2["Quiz Score"].mean())


if __name__ == "__main__":
    data_raw = pd.read_csv("quizAnswerOptions.csv", delimiter=";")

    test_data1 = pd.read_csv("ersterDurchganag.csv", delimiter=";")

    test_data2 = pd.read_csv("test_zweiterDurchganag.csv", delimiter=";")

    path = "/homes/21wedmann/colcon_ws/src/bitbots_main/bitbots_misc/bitbots_education/scripts/logs"
    dir_list = os.listdir(path)

    eval = StudyEvaluation(test_data1, test_data2, data_raw, dir_list)
    print(eval.evaluate_quiz_scores())
    print(eval.log_eval.button_dict, eval.log_eval.df)

    eval.log_eval.df.sort_values(by=["VP"], inplace=True)
    eval.log_eval.df.to_csv("evaluation_output.csv")

    """ This code block is used to visualize the quiz scores using a box plot.
    fig, axs = plt.subplots(figsize=(4,4))
    eval.quiz_data['Quiz Score'].plot.box(ax=axs)
    axs.set_ylabel("Quiz Score")
    axs.set_yticks(range(0, 16, 1))
    plt.show()
    eval.quiz_data[['Quiz Score']].plot.box()
    plt.show() """

