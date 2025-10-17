import pandas as pd
import quiz_score as qs
import sus_score as ss
import ios_score as io
import log_evaluation as le
import scipy.stats as stats
import matplotlib.pyplot as plt
import os


class StudyEvaluation:
    def __init__(self, audio_condition1_data, audio_condition2_data, web_condition1_data, web_condition2_data, quiz_data, logs_list): 
        # Assert that the ID columns are the same in both study datasets
        assert audio_condition1_data["Demographic00. Gib deine ID ei.. "].equals(
            audio_condition2_data["Demographic00. Gib deine ID ei.. "]), "ID columns in study datasets do not match."
        assert web_condition1_data["Demographic00. Gib deine ID ei.. "].equals(
            web_condition2_data["Demographic00. Gib deine ID ei.. "]), "ID columns in study datasets do not match."

        #Evaluate logs
        log_data_list = []
        for log_file in logs_list:
            log_data = pd.read_csv(os.path.join("/homes/21wedmann/colcon_ws/src/bitbots_main/bitbots_misc/bitbots_education/scripts/logs", log_file))
            log_data_sorted = log_data.sort_values(by=["VP", "Timestamp"])
            log_data_sorted["Page"].fillna("dashboard", inplace=True)
            log_data_list.append(log_data_sorted)

        self.log_eval = le.LogEvaluation(log_data_list)

        self.audio1_data = self.get_quiz_and_sus_data_and_ios(audio_condition1_data, study="study1")
        self.audio2_data = self.get_quiz_and_sus_data_and_ios(audio_condition2_data, study="study2")
        self.web1_data = self.get_quiz_and_sus_data_and_ios(web_condition1_data, study="study1")
        self.web2_data = self.get_quiz_and_sus_data_and_ios(web_condition2_data, study="study2")

        quiz_score = qs.QuizScore(quiz_data, "quiz")
        self.quiz_data = quiz_score.quiz_data

        self.significance_data = pd.DataFrame({})
        self.descriptive_data = pd.DataFrame({})
        self.descriptive_log_data = pd.DataFrame({})

        self.evaluate_scores()
        self.make_descriptive_statistics()

    def evaluate_scores(self):
        #self.calculate_significance(pd.concat([self.web1_data['SUS Score'],self.audio2_data['SUS Score']]), pd.concat([self.audio1_data['SUS Score'],self.web2_data['SUS Score']]), is_within_subject=False)

        SUS_within_audio_stat, SUS_within_audio_p = self.calculate_significance(self.audio2_data['SUS Score'], self.audio1_data['SUS Score'], is_within_subject=True)
        SUS_within_web_stat, SUS_within_web_p = self.calculate_significance(self.web1_data['SUS Score'], self.web2_data['SUS Score'], is_within_subject=True)
        SUS_between_condition1_stat, SUS_between_condition1_p = self.calculate_significance(self.audio1_data['SUS Score'], self.web1_data['SUS Score'], is_within_subject=False)
        SUS_within_study_stat, SUS_within_study_p = self.calculate_significance(pd.concat([self.web1_data['SUS Score'], self.audio2_data['SUS Score']]),
                                                                                             pd.concat([self.audio1_data['SUS Score'], self.web2_data['SUS Score']]), is_within_subject=True)


        Quiz_between_baseline_audio_stat, Quiz_between_baseline_audio_p = self.calculate_significance(self.audio1_data['Quiz Score'], self.quiz_data['Quiz Score'], is_within_subject=False)
        Quiz_between_baseline_web_stat, Quiz_between_baseline_web_p = self.calculate_significance(self.web1_data['Quiz Score'], self.quiz_data['Quiz Score'], is_within_subject=False)

        Quiz_within_audio_stat, Quiz_within_audio_p = self.calculate_significance(self.audio2_data['Quiz Score'], self.audio1_data['Quiz Score'], is_within_subject=True)
        Quiz_within_web_stat, Quiz_within_web_p = self.calculate_significance(self.web2_data['Quiz Score'], self.web1_data['Quiz Score'], is_within_subject=True)
        Quiz_between_condition1_stat, Quiz_between_condition1_p = self.calculate_significance(self.audio1_data['Quiz Score'], self.web1_data['Quiz Score'], is_within_subject=False)
        Quiz_within_study_stat, Quiz_within_study_p = self.calculate_significance(pd.concat([self.web1_data['Quiz Score'], self.audio2_data['Quiz Score']]),
                                                                                             pd.concat([self.audio1_data['Quiz Score'], self.web2_data['Quiz Score']]), is_within_subject=True)


        IOS_robot_within_audio_stat, IOS_robot_within_audio_p = self.calculate_significance(self.audio2_data['IOS Robot Score'], self.audio1_data['IOS Robot Score'], is_within_subject=True)
        IOS_robot_within_web_stat, IOS_robot_within_web_p = self.calculate_significance(self.web1_data['IOS Robot Score'], self.web2_data['IOS Robot Score'], is_within_subject=True)
        IOS_robot_between_condition1_stat, IOS_robot_between_condition1_p = self.calculate_significance(self.audio1_data['IOS Robot Score'], self.web1_data['IOS Robot Score'], is_within_subject=False)
        IOS_robot_within_study_stat, IOS_robot_within_study_p = self.calculate_significance(pd.concat([self.web1_data['IOS Robot Score'], self.audio2_data['IOS Robot Score']]),
                                                                                             pd.concat([self.audio1_data['IOS Robot Score'], self.web2_data['IOS Robot Score']]), is_within_subject=True)

        IOS_group_within_audio_stat, IOS_group_within_audio_p = self.calculate_significance(self.audio2_data['IOS Group Score'], self.audio1_data['IOS Group Score'], is_within_subject=True)
        IOS_group_within_web_stat, IOS_group_within_web_p = self.calculate_significance(self.web1_data['IOS Group Score'], self.web2_data['IOS Group Score'], is_within_subject=True)
        IOS_group_between_condition1_stat, IOS_group_between_condition1_p = self.calculate_significance(self.audio1_data['IOS Group Score'], self.web1_data['IOS Group Score'], is_within_subject=False)
        IOS_group_within_study_stat, IOS_group_within_study_p = self.calculate_significance(pd.concat([self.audio1_data['IOS Group Score'], self.web2_data['IOS Group Score']]),
                                                                                             pd.concat([self.web1_data['IOS Group Score'], self.audio2_data['IOS Group Score']]), is_within_subject=True)


        self.significance_data = pd.DataFrame({
            "Test": ["SUS within audio", "SUS within web", "SUS between condition 1", "SUS within study",
                     "Quiz between baseline audio", "Quiz between baseline web", 
                     "Quiz within audio", "Quiz within web", "Quiz between condition 1", "Quiz within study",
                     "IOS robot within audio", "IOS robot within web", "IOS robot between condition 1", "IOS robot within study",
                     "IOS group within audio", "IOS group within web", "IOS group between condition 1", "IOS group within study"],
            "Statistic": [SUS_within_audio_stat, SUS_within_web_stat, SUS_between_condition1_stat, SUS_within_study_stat,
                          Quiz_between_baseline_audio_stat, Quiz_between_baseline_web_stat, 
                          Quiz_within_audio_stat, Quiz_within_web_stat, Quiz_between_condition1_stat, Quiz_within_study_stat,
                          IOS_robot_within_audio_stat, IOS_robot_within_web_stat, IOS_robot_between_condition1_stat, IOS_robot_within_study_stat,
                          IOS_group_within_audio_stat, IOS_group_within_web_stat, IOS_group_between_condition1_stat, IOS_group_within_study_stat],
            "p-value": [SUS_within_audio_p, SUS_within_web_p, SUS_between_condition1_p, SUS_within_study_p,
                        Quiz_between_baseline_audio_p, Quiz_between_baseline_web_p, 
                        Quiz_within_audio_p, Quiz_within_web_p, Quiz_between_condition1_p, Quiz_within_study_p,
                        IOS_robot_within_audio_p, IOS_robot_within_web_p, IOS_robot_between_condition1_p, IOS_robot_within_study_p,
                        IOS_group_within_audio_p, IOS_group_within_web_p, IOS_group_between_condition1_p, IOS_group_within_study_p]
        })

    def make_descriptive_statistics(self,):
        #describe Quiz Score
        quiz_describe = self.quiz_data['Quiz Score'].describe()
        quiz_audio1_describe = self.audio1_data['Quiz Score'].describe()
        quiz_audio2_describe = self.audio2_data['Quiz Score'].describe()
        quiz_web1_describe = self.web1_data['Quiz Score'].describe()
        quiz_web2_describe = self.web2_data['Quiz Score'].describe()

        #describe SUS Score
        sus_audio1_describe = self.audio1_data['SUS Score'].describe()
        sus_audio2_describe = self.audio2_data['SUS Score'].describe()
        sus_web1_describe = self.web1_data['SUS Score'].describe()
        sus_web2_describe = self.web2_data['SUS Score'].describe()

        #describe IOS Robot Score
        ios_robot_audio1_describe = self.audio1_data['IOS Robot Score'].describe()
        ios_robot_audio2_describe = self.audio2_data['IOS Robot Score'].describe()
        ios_robot_web1_describe = self.web1_data['IOS Robot Score'].describe()
        ios_robot_web2_describe = self.web2_data['IOS Robot Score'].describe()

        #describe IOS Group Score
        ios_group_audio1_describe = self.audio1_data['IOS Group Score'].describe()
        ios_group_audio2_describe = self.audio2_data['IOS Group Score'].describe()
        ios_group_web1_describe = self.web1_data['IOS Group Score'].describe()
        ios_group_web2_describe = self.web2_data['IOS Group Score'].describe()

        self.descriptive_data = pd.concat([quiz_describe.to_frame(name="Online Quiz"),
                                           quiz_audio1_describe.to_frame(name="Audio Condition 1 Quiz"),
                                           quiz_audio2_describe.to_frame(name="Audio Condition 2 Quiz"),
                                           quiz_web1_describe.to_frame(name="Web Condition 1 Quiz"),
                                           quiz_web2_describe.to_frame(name="Web Condition 2 Quiz"),
                                           sus_audio1_describe.to_frame(name="Audio Condition 1 SUS"),
                                           sus_audio2_describe.to_frame(name="Audio Condition 2 SUS"),
                                           sus_web1_describe.to_frame(name="Web Condition 1 SUS"),
                                           sus_web2_describe.to_frame(name="Web Condition 2 SUS"),
                                           ios_robot_audio1_describe.to_frame(name="Audio Condition 1 IOS Robot"),
                                           ios_robot_audio2_describe.to_frame(name="Audio Condition 2 IOS Robot"),
                                           ios_robot_web1_describe.to_frame(name="Web Condition 1 IOS Robot"),
                                           ios_robot_web2_describe.to_frame(name="Web Condition 2 IOS Robot"),
                                           ios_group_audio1_describe.to_frame(name="Audio Condition 1 IOS Group"),
                                           ios_group_audio2_describe.to_frame(name="Audio Condition 2 IOS Group"),
                                           ios_group_web1_describe.to_frame(name="Web Condition 1 IOS Group"),
                                           ios_group_web2_describe.to_frame(name="Web Condition 2 IOS Group"),
                                           self.descriptive_data],
                                          axis=1)
        
        #describe times
        behavior_time_describe = self.log_eval.df["Behavior Time"].describe()
        motors_time_describe = self.log_eval.df["Motors Time"].describe()
        imu_time_describe = self.log_eval.df["IMU Time"].describe()
        vision_time_describe = self.log_eval.df["Vision Time"].describe()
        dashboard_time_describe = self.log_eval.df["Dashboard Time"].describe()

        #describe button presses
        txt_button_click_describe = self.log_eval.df["Text Button Clicks"].describe()
        stop_stack_button_click_describe = self.log_eval.df["Stop Stack Button Clicks"].describe()
        change_behavior_view_button_click_describe = self.log_eval.df["Change Behavior View Button Clicks"].describe()
        behavior_tree_button_click_describe = self.log_eval.df["Behavior Tree Button Clicks"].describe()

        #describe scrolled pixels
        amount_scrolled_describe = self.log_eval.df["Amount Scrolled"].describe()
        behavior_scroll_describe = self.log_eval.df["Behavior Scroll"].describe()
        motor_scroll_describe = self.log_eval.df["Motors Scroll"].describe()
        imu_scroll_describe = self.log_eval.df["IMU Scroll"].describe()
        vision_scroll_describe = self.log_eval.df["Vision Scroll"].describe()
        dashboard_scroll_describe = self.log_eval.df["Dashboard Scroll"].describe()

        self.descriptive_log_data = pd.concat([behavior_time_describe.to_frame(name="Behavior Time"),
                                              motors_time_describe.to_frame(name="Motors Time"),
                                              imu_time_describe.to_frame(name="IMU Time"),
                                              vision_time_describe.to_frame(name="Vision Time"),
                                              dashboard_time_describe.to_frame(name="Dashboard Time"),
                                              txt_button_click_describe.to_frame(name="Text Button Clicks"),
                                              stop_stack_button_click_describe.to_frame(name="Stop Stack Button Clicks"),
                                              change_behavior_view_button_click_describe.to_frame(name="Change Behavior View Button Clicks"),
                                              behavior_tree_button_click_describe.to_frame(name="Behavior Tree Button Clicks"),
                                              amount_scrolled_describe.to_frame(name="Amount Scrolled"),
                                              behavior_scroll_describe.to_frame(name="Behavior Scroll"),
                                              motor_scroll_describe.to_frame(name="Motor Scroll"),
                                              imu_scroll_describe.to_frame(name="IMU Scroll"),
                                              vision_scroll_describe.to_frame(name="Vision Scroll"),
                                              dashboard_scroll_describe.to_frame(name="Dashboard Scroll"),
                                              self.descriptive_log_data],
                                             axis=1)


    def make_quiz_histogram(self, data: pd.Series):
        # Compute min and max
        min_val = int(data.min())
        max_val = int(data.max())

        # Define bins for each integer value
        bins = range(min_val, max_val + 2) # +2 to include the last value

        plt.hist(data, bins=bins, edgecolor='black')
        plt.title("Audio Condition 2 - Quiz Score Distribution")
        plt.xlabel("Quiz Score")
        plt.ylabel("Frequency")
        plt.show()

    def get_quiz_and_sus_data_and_ios(self, data, study):
        study_score = qs.QuizScore(data, study)
        sus = ss.SUSScore(study_score.quiz_data, study)
        ios = io.IOSScore(sus.sus_data, study)
        return ios.ios_data

    def calculate_significance(self, data1, data2, is_within_subject):
        if is_within_subject:
            stat, p_value = stats.wilcoxon(data1, data2, alternative="greater")
            return stat, p_value
        else:
            stat, p_value = stats.mannwhitneyu(data1, data2, alternative="greater")
            return stat, p_value


if __name__ == "__main__":
    data_raw = pd.read_csv("quiz_answers.csv", delimiter=";")

    audio_1_condition = pd.read_csv("first_condition_audio.csv", delimiter=";")
    audio_2_condition = pd.read_csv("second_condition_audio.csv", delimiter=";")
    web_1_condition = pd.read_csv("first_condition_web.csv", delimiter=";")
    web_2_condition = pd.read_csv("second_condition_web.csv", delimiter=";")

    path = "/homes/21wedmann/colcon_ws/src/bitbots_main/bitbots_misc/bitbots_education/scripts/logs"
    dir_list = os.listdir(path)

    eval = StudyEvaluation(audio_1_condition, audio_2_condition, web_1_condition, web_2_condition, data_raw, dir_list)
    print(eval.descriptive_data)

    eval.significance_data.to_csv("significance_data.csv")
    eval.descriptive_data.to_csv("descriptive_data.csv")
    eval.descriptive_log_data.to_csv("descriptive_log_data.csv")
    # print(eval.log_eval.button_dict, eval.log_eval.df)

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

