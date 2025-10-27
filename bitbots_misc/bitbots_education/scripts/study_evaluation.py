import pandas as pd
import numpy as np
import quiz_score as qs
import sus_score as ss
import ios_score as io
import ueq_score as ue
import demografic as dm
import log_evaluation as le
import scipy.stats as stats
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os


class StudyEvaluation:
    def __init__(self, audio_condition1_data, audio_condition2_data, web_condition1_data, web_condition2_data, quiz_data): #logs_list): 
        # Assert that the ID columns are the same in both study datasets
        assert audio_condition1_data["Demographic00. Gib deine ID ei.. "].equals(
            audio_condition2_data["Demographic00. Gib deine ID ei.. "]), "ID columns in study datasets do not match."
        assert web_condition1_data["Demographic00. Gib deine ID ei.. "].equals(
            web_condition2_data["Demographic00. Gib deine ID ei.. "]), "ID columns in study datasets do not match."

        #Evaluate logs
        # log_data_list = []
        # for log_file in logs_list:
        #     log_data = pd.read_csv(os.path.join("/homes/21wedmann/colcon_ws/src/bitbots_main/bitbots_misc/bitbots_education/scripts/logs", log_file))
        #     log_data_sorted = log_data.sort_values(by=["VP", "Timestamp"])
        #     log_data_sorted["Page"].fillna("dashboard", inplace=True)
        #     log_data_list.append(log_data_sorted)

        #self.log_eval = le.LogEvaluation(log_data_list)

        self.audio1_data = self.get_quiz_and_sus_data_and_ios(audio_condition1_data, study="study1")
        self.audio2_data = self.get_quiz_and_sus_data_and_ios(audio_condition2_data, study="study2")
        self.web1_data = self.get_quiz_and_sus_data_and_ios(web_condition1_data, study="study1")
        self.web2_data = self.get_quiz_and_sus_data_and_ios(web_condition2_data, study="study2")

        quiz_score = qs.QuizScore(quiz_data, "quiz")
        self.quiz_data = quiz_score.quiz_data

        self.significance_data = pd.DataFrame({})
        self.descriptive_data = pd.DataFrame({})
        #self.descriptive_log_data = pd.DataFrame({})

        self.evaluate_scores()
        self.make_descriptive_statistics()

    def evaluate_scores(self):
        #self.calculate_significance(pd.concat([self.web1_data['SUS Score'],self.audio2_data['SUS Score']]), pd.concat([self.audio1_data['SUS Score'],self.web2_data['SUS Score']]), is_within_subject=False)

        SUS_within_audio_stat, SUS_within_audio_p, SUS_within_audio_test, SUS_within_audio_data1_normal, SUS_within_audio_data2_normal, SUS_within_audio_data1_p, SUS_within_audio_data2_p = self.calculate_significance(self.audio2_data['SUS Score'], self.audio1_data['SUS Score'], is_within_subject=True)
        SUS_within_web_stat, SUS_within_web_p, SUS_within_web_test, SUS_within_web_data1_normal, SUS_within_web_data2_normal, SUS_within_web_data1_p, SUS_within_web_data2_p = self.calculate_significance(self.web1_data['SUS Score'], self.web2_data['SUS Score'], is_within_subject=True)
        SUS_between_condition1_stat, SUS_between_condition1_p, SUS_between_condition1_test, SUS_between_condition1_data1_normal, SUS_between_condition1_data2_normal, SUS_between_condition1_data1_p, SUS_between_condition1_data2_p = self.calculate_significance(self.audio1_data['SUS Score'], self.web1_data['SUS Score'], is_within_subject=False)
        SUS_within_study_stat, SUS_within_study_p, SUS_within_study_test, SUS_within_study_data1_normal, SUS_within_study_data2_normal, SUS_within_study_data1_p, SUS_within_study_data2_p = self.calculate_significance(pd.concat([self.web1_data['SUS Score'], self.audio2_data['SUS Score']]),
                                                                                             pd.concat([self.audio1_data['SUS Score'], self.web2_data['SUS Score']]), is_within_subject=True)


        Quiz_between_baseline_audio_stat, Quiz_between_baseline_audio_p, Quiz_between_baseline_audio_test, Quiz_between_baseline_audio_data1_normal, Quiz_between_baseline_audio_data1_p, Quiz_between_baseline_audio_data2_normal, Quiz_between_baseline_audio_data2_p = self.calculate_significance(self.audio1_data['Quiz Score'], self.quiz_data['Quiz Score'], is_within_subject=False)
        Quiz_between_baseline_web_stat, Quiz_between_baseline_web_p, Quiz_between_baseline_web_test, Quiz_between_baseline_web_data1_normal, Quiz_between_baseline_web_data1_p, Quiz_between_baseline_web_data2_normal, Quiz_between_baseline_web_data2_p = self.calculate_significance(self.web1_data['Quiz Score'], self.quiz_data['Quiz Score'], is_within_subject=False)

        Quiz_within_audio_stat, Quiz_within_audio_p, Quiz_within_audio_test, Quiz_within_audio_data1_normal, Quiz_within_audio_data2_normal, Quiz_within_audio_data1_p, Quiz_within_audio_data2_p = self.calculate_significance(self.audio2_data['Quiz Score'], self.audio1_data['Quiz Score'], is_within_subject=True)
        Quiz_within_web_stat, Quiz_within_web_p, Quiz_within_web_test, Quiz_within_web_data1_normal, Quiz_within_web_data2_normal, Quiz_within_web_data1_p, Quiz_within_web_data2_p = self.calculate_significance(self.web2_data['Quiz Score'], self.web1_data['Quiz Score'], is_within_subject=True)
        Quiz_between_condition1_stat, Quiz_between_condition1_p, Quiz_between_condition1_test, Quiz_between_condition1_data1_normal, Quiz_between_condition1_data2_normal, Quiz_between_condition1_data1_p, Quiz_between_condition1_data2_p = self.calculate_significance(self.audio1_data['Quiz Score'], self.web1_data['Quiz Score'], is_within_subject=False)
        Quiz_within_study_stat, Quiz_within_study_p, Quiz_within_study_test, Quiz_within_study_data1_normal, Quiz_within_study_data2_normal, Quiz_within_study_data1_p, Quiz_within_study_data2_p = self.calculate_significance(pd.concat([self.web1_data['Quiz Score'], self.audio2_data['Quiz Score']]),
                                                                                             pd.concat([self.audio1_data['Quiz Score'], self.web2_data['Quiz Score']]), is_within_subject=True)


        IOS_robot_within_audio_stat, IOS_robot_within_audio_p, IOS_robot_within_audio_test, IOS_robot_within_audio_data1_normal, IOS_robot_within_audio_data2_normal, IOS_robot_within_audio_data1_p, IOS_robot_within_audio_data2_p = self.calculate_significance(self.audio2_data['IOS Robot Score'], self.audio1_data['IOS Robot Score'], is_within_subject=True)
        IOS_robot_within_web_stat, IOS_robot_within_web_p, IOS_robot_within_web_test, IOS_robot_within_web_data1_normal, IOS_robot_within_web_data2_normal, IOS_robot_within_web_data1_p, IOS_robot_within_web_data2_p = self.calculate_significance(self.web1_data['IOS Robot Score'], self.web2_data['IOS Robot Score'], is_within_subject=True)
        IOS_robot_between_condition1_stat, IOS_robot_between_condition1_p, IOS_robot_between_condition1_test, IOS_robot_between_condition1_data1_normal, IOS_robot_between_condition1_data2_normal, IOS_robot_between_condition1_data1_p, IOS_robot_between_condition1_data2_p = self.calculate_significance(self.audio1_data['IOS Robot Score'], self.web1_data['IOS Robot Score'], is_within_subject=False)
        IOS_robot_within_study_stat, IOS_robot_within_study_p, IOS_robot_within_study_test, IOS_robot_within_study_data1_normal, IOS_robot_within_study_data2_normal, IOS_robot_within_study_data1_p, IOS_robot_within_study_data2_p = self.calculate_significance(pd.concat([self.web1_data['IOS Robot Score'], self.audio2_data['IOS Robot Score']]),
                                                                                             pd.concat([self.audio1_data['IOS Robot Score'], self.web2_data['IOS Robot Score']]), is_within_subject=True)

        IOS_group_within_audio_stat, IOS_group_within_audio_p, IOS_group_within_audio_test, IOS_group_within_audio_data1_normal, IOS_group_within_audio_data2_normal, IOS_group_within_audio_data1_p, IOS_group_within_audio_data2_p = self.calculate_significance(self.audio2_data['IOS Group Score'], self.audio1_data['IOS Group Score'], is_within_subject=True)
        IOS_group_within_web_stat, IOS_group_within_web_p, IOS_group_within_web_test, IOS_group_within_web_data1_normal, IOS_group_within_web_data2_normal, IOS_group_within_web_data1_p, IOS_group_within_web_data2_p = self.calculate_significance(self.web1_data['IOS Group Score'], self.web2_data['IOS Group Score'], is_within_subject=True)
        IOS_group_between_condition1_stat, IOS_group_between_condition1_p, IOS_group_between_condition1_test, IOS_group_between_condition1_data1_normal, IOS_group_between_condition1_data2_normal, IOS_group_between_condition1_data1_p, IOS_group_between_condition1_data2_p = self.calculate_significance(self.audio1_data['IOS Group Score'], self.web1_data['IOS Group Score'], is_within_subject=False)
        IOS_group_within_study_stat, IOS_group_within_study_p, IOS_group_within_study_test, IOS_group_within_study_data1_normal, IOS_group_within_study_data2_normal, IOS_group_within_study_data1_p, IOS_group_within_study_data2_p = self.calculate_significance(pd.concat([self.audio1_data['IOS Group Score'], self.web2_data['IOS Group Score']]),
                                                                                             pd.concat([self.web1_data['IOS Group Score'], self.audio2_data['IOS Group Score']]), is_within_subject=True)
        UEQ_pragmatic_within_audio_stat, UEQ_pragmatic_within_audio_p, UEQ_pragmatic_within_audio_test, UEQ_pragmatic_within_audio_data1_normal, UEQ_pragmatic_within_audio_data2_normal, UEQ_pragmatic_within_audio_data1_p, UEQ_pragmatic_within_audio_data2_p = self.calculate_significance(self.audio2_data['UEQs Pragmatic Score'], self.audio1_data['UEQs Pragmatic Score'], is_within_subject=True)
        UEQ_pragmatic_within_web_stat, UEQ_pragmatic_within_web_p, UEQ_pragmatic_within_web_test, UEQ_pragmatic_within_web_data1_normal, UEQ_pragmatic_within_web_data2_normal, UEQ_pragmatic_within_web_data1_p, UEQ_pragmatic_within_web_data2_p = self.calculate_significance(self.web1_data['UEQs Pragmatic Score'], self.web2_data['UEQs Pragmatic Score'], is_within_subject=True)
        UEQ_pragmatic_between_condition1_stat, UEQ_pragmatic_between_condition1_p, UEQ_pragmatic_between_condition1_test, UEQ_pragmatic_between_condition1_data1_normal, UEQ_pragmatic_between_condition1_data2_normal, UEQ_pragmatic_between_condition1_data1_p, UEQ_pragmatic_between_condition1_data2_p = self.calculate_significance(self.web1_data['UEQs Pragmatic Score'], self.audio1_data['UEQs Pragmatic Score'], is_within_subject=False)
        UEQ_pragmatic_within_study_stat, UEQ_pragmatic_within_study_p, UEQ_pragmatic_within_study_test, UEQ_pragmatic_within_study_data1_normal, UEQ_pragmatic_within_study_data2_normal, UEQ_pragmatic_within_study_data1_p, UEQ_pragmatic_within_study_data2_p = self.calculate_significance(pd.concat([self.web1_data['UEQs Pragmatic Score'], self.audio2_data['UEQs Pragmatic Score']])
                                                                                                                                     ,pd.concat([self.audio1_data['UEQs Pragmatic Score'], self.web2_data['UEQs Pragmatic Score']]), is_within_subject=True)

        UEQ_hedonic_within_audio_stat, UEQ_hedonic_within_audio_p, UEQ_hedonic_within_audio_test, UEQ_hedonic_within_audio_data1_normal, UEQ_hedonic_within_audio_data2_normal, UEQ_hedonic_within_audio_data1_p, UEQ_hedonic_within_audio_data2_p = self.calculate_significance(self.audio2_data['UEQs Hedonic Score'], self.audio1_data['UEQs Hedonic Score'], is_within_subject=True)
        UEQ_hedonic_within_web_stat, UEQ_hedonic_within_web_p, UEQ_hedonic_within_web_test, UEQ_hedonic_within_web_data1_normal, UEQ_hedonic_within_web_data2_normal, UEQ_hedonic_within_web_data1_p, UEQ_hedonic_within_web_data2_p = self.calculate_significance(self.web1_data['UEQs Hedonic Score'], self.web2_data['UEQs Hedonic Score'], is_within_subject=True)
        UEQ_hedonic_between_condition1_stat, UEQ_hedonic_between_condition1_p, UEQ_hedonic_between_condition1_test, UEQ_hedonic_between_condition1_data1_normal, UEQ_hedonic_between_condition1_data2_normal, UEQ_hedonic_between_condition1_data1_p, UEQ_hedonic_between_condition1_data2_p = self.calculate_significance(self.web1_data['UEQs Hedonic Score'], self.audio1_data['UEQs Hedonic Score'], is_within_subject=False)
        UEQ_hedonic_within_study_stat, UEQ_hedonic_within_study_p, UEQ_hedonic_within_study_test, UEQ_hedonic_within_study_data1_normal, UEQ_hedonic_within_study_data2_normal, UEQ_hedonic_within_study_data1_p, UEQ_hedonic_within_study_data2_p = self.calculate_significance(pd.concat([self.web1_data['UEQs Hedonic Score'], self.audio2_data['UEQs Hedonic Score']])
                                                                                                                                ,pd.concat([self.audio1_data['UEQs Hedonic Score'], self.web2_data['UEQs Hedonic Score']]), is_within_subject=True)

        UEQ_overall_within_audio_stat, UEQ_overall_within_audio_p, UEQ_overall_within_audio_test, UEQ_overall_within_audio_data1_normal, UEQ_overall_within_audio_data2_normal, UEQ_overall_within_audio_data1_p, UEQ_overall_within_audio_data2_p = self.calculate_significance(self.audio2_data['UEQs Overall Score'], self.audio1_data['UEQs Overall Score'], is_within_subject=True)
        UEQ_overall_within_web_stat, UEQ_overall_within_web_p, UEQ_overall_within_web_test, UEQ_overall_within_web_data1_normal, UEQ_overall_within_web_data2_normal, UEQ_overall_within_web_data1_p, UEQ_overall_within_web_data2_p = self.calculate_significance(self.web1_data['UEQs Overall Score'], self.web2_data['UEQs Overall Score'], is_within_subject=True)
        UEQ_overall_between_condition1_stat, UEQ_overall_between_condition1_p, UEQ_overall_between_condition1_test, UEQ_overall_between_condition1_data1_normal, UEQ_overall_between_condition1_data2_normal, UEQ_overall_between_condition1_data1_p, UEQ_overall_between_condition1_data2_p = self.calculate_significance(self.web1_data['UEQs Overall Score'], self.audio1_data['UEQs Overall Score'], is_within_subject=False)
        UEQ_overall_within_study_stat, UEQ_overall_within_study_p, UEQ_overall_within_study_test, UEQ_overall_within_study_data1_normal, UEQ_overall_within_study_data2_normal, UEQ_overall_within_study_data1_p, UEQ_overall_within_study_data2_p = self.calculate_significance(pd.concat([self.web1_data['UEQs Overall Score'], self.audio2_data['UEQs Overall Score']])
                                                                                                                                ,pd.concat([self.audio1_data['UEQs Overall Score'], self.web2_data['UEQs Overall Score']]), is_within_subject=True)

        self.significance_data = pd.DataFrame({
            "Test": ["SUS within audio", "SUS within web", "SUS between condition 1", "SUS within study", 
                     "Quiz between baseline audio", "Quiz between baseline web",
                     "Quiz within audio", "Quiz within web", "Quiz between condition 1", "Quiz within study",
                     "IOS robot within audio", "IOS robot within web", "IOS robot between condition 1", "IOS robot within study",
                     "IOS group within audio", "IOS group within web", "IOS group between condition 1", "IOS group within study",
                     "UEQ pragmatic within audio", "UEQ pragmatic within web", "UEQ pragmatic between condition 1", "UEQ pragmatic within study",
                    "UEQ hedonic within audio", "UEQ hedonic within web", "UEQ hedonic between condition 1", "UEQ hedonic within study", 
                    "UEQ overall within audio", "UEQ overall within web", "UEQ overall between condition 1", "UEQ overall within study", ],
            "Statistic": [SUS_within_audio_stat, SUS_within_web_stat, SUS_between_condition1_stat, SUS_within_study_stat, 
                          Quiz_between_baseline_audio_stat, Quiz_between_baseline_web_stat, 
                          Quiz_within_audio_stat, Quiz_within_web_stat, Quiz_between_condition1_stat, Quiz_within_study_stat,
                          IOS_robot_within_audio_stat, IOS_robot_within_web_stat, IOS_robot_between_condition1_stat, IOS_robot_within_study_stat,
                          IOS_group_within_audio_stat, IOS_group_within_web_stat, IOS_group_between_condition1_stat, IOS_group_within_study_stat,
                          UEQ_pragmatic_within_audio_stat, UEQ_pragmatic_within_web_stat, UEQ_pragmatic_between_condition1_stat, UEQ_pragmatic_within_study_stat,
                          UEQ_hedonic_within_audio_stat, UEQ_hedonic_within_web_stat, UEQ_hedonic_between_condition1_stat, UEQ_hedonic_within_study_stat,
                          UEQ_overall_within_audio_stat, UEQ_overall_within_web_stat, UEQ_overall_between_condition1_stat, UEQ_overall_within_study_stat],
            "p-value": [SUS_within_audio_p, SUS_within_web_p, SUS_between_condition1_p, SUS_within_study_p,
                        Quiz_between_baseline_audio_p, Quiz_between_baseline_web_p, 
                        Quiz_within_audio_p, Quiz_within_web_p, Quiz_between_condition1_p, Quiz_within_study_p,
                        IOS_robot_within_audio_p, IOS_robot_within_web_p, IOS_robot_between_condition1_p, IOS_robot_within_study_p,
                        IOS_group_within_audio_p, IOS_group_within_web_p, IOS_group_between_condition1_p, IOS_group_within_study_p,
                        UEQ_pragmatic_within_audio_p, UEQ_pragmatic_within_web_p, UEQ_pragmatic_between_condition1_p, UEQ_pragmatic_within_study_p,
                        UEQ_hedonic_within_audio_p, UEQ_hedonic_within_web_p, UEQ_hedonic_between_condition1_p, UEQ_hedonic_within_study_p,
                        UEQ_overall_within_audio_p, UEQ_overall_within_web_p, UEQ_overall_between_condition1_p, UEQ_overall_within_study_p],    
            "test statistic": [SUS_within_audio_stat, SUS_within_web_stat, SUS_between_condition1_stat, SUS_within_study_stat,
                               Quiz_between_baseline_audio_stat, Quiz_between_baseline_web_stat, 
                               Quiz_within_audio_stat, Quiz_within_web_stat, Quiz_between_condition1_stat, Quiz_within_study_stat,
                               IOS_robot_within_audio_stat, IOS_robot_within_web_stat, IOS_robot_between_condition1_stat, IOS_robot_within_study_stat,
                               IOS_group_within_audio_stat, IOS_group_within_web_stat, IOS_group_between_condition1_stat, IOS_group_within_study_stat,
                               UEQ_pragmatic_within_audio_stat, UEQ_pragmatic_within_web_stat, UEQ_pragmatic_between_condition1_stat, UEQ_pragmatic_within_study_stat,
                               UEQ_hedonic_within_audio_stat, UEQ_hedonic_within_web_stat, UEQ_hedonic_between_condition1_stat, UEQ_hedonic_within_study_stat,
                               UEQ_overall_within_audio_stat, UEQ_overall_within_web_stat, UEQ_overall_between_condition1_stat, UEQ_overall_within_study_stat],          
            "test type":[SUS_within_audio_test, SUS_within_web_test, SUS_between_condition1_test, SUS_within_study_test,
                          Quiz_between_baseline_audio_test, Quiz_between_baseline_web_test, 
                          Quiz_within_audio_test, Quiz_within_web_test, Quiz_between_condition1_test, Quiz_within_study_test,
                          IOS_robot_within_audio_test, IOS_robot_within_web_test, IOS_robot_between_condition1_test, IOS_robot_within_study_test,
                          IOS_group_within_audio_test, IOS_group_within_web_test, IOS_group_between_condition1_test, IOS_group_within_study_test,
                          UEQ_pragmatic_within_audio_test, UEQ_pragmatic_within_web_test, UEQ_pragmatic_between_condition1_test, UEQ_pragmatic_within_study_test,
                          UEQ_hedonic_within_audio_test, UEQ_hedonic_within_web_test, UEQ_hedonic_between_condition1_test, UEQ_hedonic_within_study_test,
                          UEQ_overall_within_audio_test, UEQ_overall_within_web_test, UEQ_overall_between_condition1_test, UEQ_overall_within_study_test],
            "data1 normal": [SUS_within_audio_data1_normal, SUS_within_web_data1_normal, SUS_between_condition1_data1_normal, SUS_within_study_data1_normal,
                             Quiz_between_baseline_audio_data1_normal, Quiz_between_baseline_web_data1_normal,
                             Quiz_within_audio_data1_normal, Quiz_within_web_data1_normal, Quiz_between_condition1_data1_normal, Quiz_within_study_data1_normal,
                             IOS_robot_within_audio_data1_normal, IOS_robot_within_web_data1_normal, IOS_robot_between_condition1_data1_normal, IOS_robot_within_study_data1_normal,
                             IOS_group_within_audio_data1_normal, IOS_group_within_web_data1_normal, IOS_group_between_condition1_data1_normal, IOS_group_within_study_data1_normal,
                             UEQ_pragmatic_within_audio_data1_normal, UEQ_pragmatic_within_web_data1_normal, UEQ_pragmatic_between_condition1_data1_normal, UEQ_pragmatic_within_study_data1_normal,
                             UEQ_hedonic_within_audio_data1_normal, UEQ_hedonic_within_web_data1_normal, UEQ_hedonic_between_condition1_data1_normal, UEQ_hedonic_within_study_data1_normal,
                             UEQ_overall_within_audio_data1_normal, UEQ_overall_within_web_data1_normal, UEQ_overall_between_condition1_data1_normal, UEQ_overall_within_study_data1_normal],
            "data1 p-value": [SUS_within_audio_data1_p, SUS_within_web_data1_p, SUS_between_condition1_data1_p, SUS_within_study_data1_p,
                              Quiz_between_baseline_audio_data1_p, Quiz_between_baseline_web_data1_p,
                              Quiz_within_audio_data1_p, Quiz_within_web_data1_p, Quiz_between_condition1_data1_p, Quiz_within_study_data1_p,
                              IOS_robot_within_audio_data1_p, IOS_robot_within_web_data1_p, IOS_robot_between_condition1_data1_p, IOS_robot_within_study_data1_p,
                              IOS_group_within_audio_data1_p, IOS_group_within_web_data1_p, IOS_group_between_condition1_data1_p, IOS_group_within_study_data1_p,
                              UEQ_pragmatic_within_audio_data1_p, UEQ_pragmatic_within_web_data1_p, UEQ_pragmatic_between_condition1_data1_p, UEQ_pragmatic_within_study_data1_p,
                              UEQ_hedonic_within_audio_data1_p, UEQ_hedonic_within_web_data1_p, UEQ_hedonic_between_condition1_data1_p, UEQ_hedonic_within_study_data1_p,
                              UEQ_overall_within_audio_data1_p, UEQ_overall_within_web_data1_p, UEQ_overall_between_condition1_data1_p, UEQ_overall_within_study_data1_p],
            "data2 normal": [SUS_within_audio_data2_normal, SUS_within_web_data2_normal, SUS_between_condition1_data2_normal, SUS_within_study_data2_normal,
                             Quiz_between_baseline_audio_data2_normal, Quiz_between_baseline_web_data2_normal,
                             Quiz_within_audio_data2_normal, Quiz_within_web_data2_normal, Quiz_between_condition1_data2_normal, Quiz_within_study_data2_normal,
                             IOS_robot_within_audio_data2_normal, IOS_robot_within_web_data2_normal, IOS_robot_between_condition1_data2_normal, IOS_robot_within_study_data2_normal,
                             IOS_group_within_audio_data2_normal, IOS_group_within_web_data2_normal, IOS_group_between_condition1_data2_normal, IOS_group_within_study_data2_normal,
                             UEQ_pragmatic_within_audio_data2_normal, UEQ_pragmatic_within_web_data2_normal, UEQ_pragmatic_between_condition1_data2_normal, UEQ_pragmatic_within_study_data2_normal,
                             UEQ_hedonic_within_audio_data2_normal, UEQ_hedonic_within_web_data2_normal, UEQ_hedonic_between_condition1_data2_normal, UEQ_hedonic_within_study_data2_normal,
                             UEQ_overall_within_audio_data2_normal, UEQ_overall_within_web_data2_normal, UEQ_overall_between_condition1_data2_normal, UEQ_overall_within_study_data2_normal],
            "data2 p-value": [SUS_within_audio_data2_p, SUS_within_web_data2_p, SUS_between_condition1_data2_p, SUS_within_study_data2_p,
                              Quiz_between_baseline_audio_data2_p, Quiz_between_baseline_web_data2_p,
                              Quiz_within_audio_data2_p, Quiz_within_web_data2_p, Quiz_between_condition1_data2_p, Quiz_within_study_data2_p,
                              IOS_robot_within_audio_data2_p, IOS_robot_within_web_data2_p, IOS_robot_between_condition1_data2_p, IOS_robot_within_study_data2_p,
                              IOS_group_within_audio_data2_p, IOS_group_within_web_data2_p, IOS_group_between_condition1_data2_p, IOS_group_within_study_data2_p,
                              UEQ_pragmatic_within_audio_data2_p, UEQ_pragmatic_within_web_data2_p, UEQ_pragmatic_between_condition1_data2_p, UEQ_pragmatic_within_study_data2_p,
                              UEQ_hedonic_within_audio_data2_p, UEQ_hedonic_within_web_data2_p, UEQ_hedonic_between_condition1_data2_p, UEQ_hedonic_within_study_data2_p,
                              UEQ_overall_within_audio_data2_p, UEQ_overall_within_web_data2_p, UEQ_overall_between_condition1_data2_p, UEQ_overall_within_study_data2_p]

        })

    def make_descriptive_statistics(self,):
        #describe Quiz Score
        quiz_describe = self.quiz_data['Quiz Score'].describe()
        quiz_audio1_describe = self.audio1_data['Quiz Score'].describe()
        quiz_audio2_describe = self.audio2_data['Quiz Score'].describe()
        quiz_web1_describe = self.web1_data['Quiz Score'].describe()
        quiz_web2_describe = self.web2_data['Quiz Score'].describe()
        quiz_all_audio_describe = pd.concat([self.audio1_data['Quiz Score'], self.web2_data['Quiz Score']]).describe()
        quiz_all_web_describe = pd.concat([self.web1_data['Quiz Score'], self.audio2_data['Quiz Score']]).describe()

        #describe SUS Score
        sus_audio1_describe = self.audio1_data['SUS Score'].describe()
        sus_audio2_describe = self.audio2_data['SUS Score'].describe()
        sus_web1_describe = self.web1_data['SUS Score'].describe()
        sus_web2_describe = self.web2_data['SUS Score'].describe()
        sus_all_audio_describe = pd.concat([self.audio1_data['SUS Score'], self.web2_data['SUS Score']]).describe()
        sus_all_web_describe = pd.concat([self.web1_data['SUS Score'], self.audio2_data['SUS Score']]).describe()

        #describe IOS Robot Score
        ios_robot_audio1_describe = self.audio1_data['IOS Robot Score'].describe()
        ios_robot_audio2_describe = self.audio2_data['IOS Robot Score'].describe()
        ios_robot_web1_describe = self.web1_data['IOS Robot Score'].describe()
        ios_robot_web2_describe = self.web2_data['IOS Robot Score'].describe()
        ios_robot_all_audio_describe = pd.concat([self.audio1_data['IOS Robot Score'], self.web2_data['IOS Robot Score']]).describe()
        ios_robot_all_web_describe = pd.concat([self.web1_data['IOS Robot Score'], self.audio2_data['IOS Robot Score']]).describe()

        #describe IOS Group Score
        ios_group_audio1_describe = self.audio1_data['IOS Group Score'].describe()
        ios_group_audio2_describe = self.audio2_data['IOS Group Score'].describe()
        ios_group_web1_describe = self.web1_data['IOS Group Score'].describe()
        ios_group_web2_describe = self.web2_data['IOS Group Score'].describe()
        ios_group_all_audio_describe = pd.concat([self.audio1_data['IOS Group Score'], self.web2_data['IOS Group Score']]).describe()
        ios_group_all_web_describe = pd.concat([self.web1_data['IOS Group Score'], self.audio2_data['IOS Group Score']]).describe()

        #describe UEQ Scores
        #pragmatic
        ueq_pragmatic_audio1_describe = self.audio1_data['UEQs Pragmatic Score'].describe()
        ueq_pragmatic_audio2_describe = self.audio2_data['UEQs Pragmatic Score'].describe()
        ueq_pragmatic_web1_describe = self.web1_data['UEQs Pragmatic Score'].describe()
        ueq_pragmatic_web2_describe = self.web2_data['UEQs Pragmatic Score'].describe()
        ueq_pragmatic_all_audio_describe = pd.concat([self.audio1_data['UEQs Pragmatic Score'], self.web2_data['UEQs Pragmatic Score']]).describe()
        ueq_pragmatic_all_web_describe = pd.concat([self.web1_data['UEQs Pragmatic Score'], self.audio2_data['UEQs Pragmatic Score']]).describe()

        #hedonic
        ueq_hedonic_audio1_describe = self.audio1_data['UEQs Hedonic Score'].describe()
        ueq_hedonic_audio2_describe = self.audio2_data['UEQs Hedonic Score'].describe()
        ueq_hedonic_web1_describe = self.web1_data['UEQs Hedonic Score'].describe()
        ueq_hedonic_web2_describe = self.web2_data['UEQs Hedonic Score'].describe()
        ueq_hedonic_all_audio_describe = pd.concat([self.audio1_data['UEQs Hedonic Score'], self.web2_data['UEQs Hedonic Score']]).describe()
        ueq_hedonic_all_web_describe = pd.concat([self.web1_data['UEQs Hedonic Score'], self.audio2_data['UEQs Hedonic Score']]).describe()

        #overall
        ueq_overall_audio1_describe = self.audio1_data['UEQs Overall Score'].describe()
        ueq_overall_audio2_describe = self.audio2_data['UEQs Overall Score'].describe()
        ueq_overall_web1_describe = self.web1_data['UEQs Overall Score'].describe()
        ueq_overall_web2_describe = self.web2_data['UEQs Overall Score'].describe()
        ueq_overall_all_audio_describe = pd.concat([self.audio1_data['UEQs Overall Score'], self.web2_data['UEQs Overall Score']]).describe()
        ueq_overall_all_web_describe = pd.concat([self.web1_data['UEQs Overall Score'], self.audio2_data['UEQs Overall Score']]).describe()

        self.descriptive_data = pd.concat([quiz_describe.to_frame(name="Online Quiz"),
                                           quiz_audio1_describe.to_frame(name="Audio Condition 1 Quiz"),
                                           quiz_audio2_describe.to_frame(name="Audio Condition 2 Quiz"),
                                           quiz_web1_describe.to_frame(name="Web Condition 1 Quiz"),
                                           quiz_web2_describe.to_frame(name="Web Condition 2 Quiz"),
                                           quiz_all_audio_describe.to_frame(name="All Audio Quiz"),
                                           quiz_all_web_describe.to_frame(name="All Web Quiz"),   
                                           sus_audio1_describe.to_frame(name="Audio Condition 1 SUS"),
                                           sus_audio2_describe.to_frame(name="Audio Condition 2 SUS"),
                                           sus_web1_describe.to_frame(name="Web Condition 1 SUS"),
                                           sus_web2_describe.to_frame(name="Web Condition 2 SUS"),
                                           sus_all_audio_describe.to_frame(name="All Audio SUS"),
                                           sus_all_web_describe.to_frame(name="All Web SUS"),
                                           ios_robot_audio1_describe.to_frame(name="Audio Condition 1 IOS Robot"),
                                           ios_robot_audio2_describe.to_frame(name="Audio Condition 2 IOS Robot"),
                                           ios_robot_web1_describe.to_frame(name="Web Condition 1 IOS Robot"),
                                           ios_robot_web2_describe.to_frame(name="Web Condition 2 IOS Robot"),
                                           ios_robot_all_audio_describe.to_frame(name="All Audio IOS Robot"),
                                           ios_robot_all_web_describe.to_frame(name="All Web IOS Robot"),
                                           ios_group_audio1_describe.to_frame(name="Audio Condition 1 IOS Group"),
                                           ios_group_audio2_describe.to_frame(name="Audio Condition 2 IOS Group"),
                                           ios_group_web1_describe.to_frame(name="Web Condition 1 IOS Group"),
                                           ios_group_web2_describe.to_frame(name="Web Condition 2 IOS Group"),
                                           ios_group_all_audio_describe.to_frame(name="All Audio IOS Group"),
                                           ios_group_all_web_describe.to_frame(name="All Web IOS Group"),
                                           ueq_pragmatic_audio1_describe.to_frame(name="Audio Condition 1 UEQ Pragmatic"),
                                           ueq_pragmatic_audio2_describe.to_frame(name="Audio Condition 2 UEQ Pragmatic"),
                                           ueq_pragmatic_web1_describe.to_frame(name="Web Condition 1 UEQ Pragmatic"),
                                           ueq_pragmatic_web2_describe.to_frame(name="Web Condition 2 UEQ Pragmatic"),
                                           ueq_pragmatic_all_audio_describe.to_frame(name="All Audio UEQ Pragmatic"),
                                           ueq_pragmatic_all_web_describe.to_frame(name="All Web UEQ Pragmatic"),
                                           ueq_hedonic_audio1_describe.to_frame(name="Audio Condition 1 UEQ Hedonic"),
                                           ueq_hedonic_audio2_describe.to_frame(name="Audio Condition 2 UEQ Hedonic"),
                                           ueq_hedonic_web1_describe.to_frame(name="Web Condition 1 UEQ Hedonic"),
                                           ueq_hedonic_web2_describe.to_frame(name="Web Condition 2 UEQ Hedonic"),
                                           ueq_hedonic_all_audio_describe.to_frame(name="All Audio UEQ Hedonic"),
                                           ueq_hedonic_all_web_describe.to_frame(name="All Web UEQ Hedonic"),
                                           ueq_overall_audio1_describe.to_frame(name="Audio Condition 1 UEQ Overall"),
                                           ueq_overall_audio2_describe.to_frame(name="Audio Condition 2 UEQ Overall"),
                                           ueq_overall_web1_describe.to_frame(name="Web Condition 1 UEQ Overall"),
                                           ueq_overall_web2_describe.to_frame(name="Web Condition 2 UEQ Overall"),
                                           ueq_overall_all_audio_describe.to_frame(name="All Audio UEQ Overall"),
                                           ueq_overall_all_web_describe.to_frame(name="All Web UEQ Overall"),
                                           self.descriptive_data],
                                          axis=1)
        
        # #describe times
        # behavior_time_describe = self.log_eval.df["Behavior Time"].describe()
        # motors_time_describe = self.log_eval.df["Motors Time"].describe()
        # imu_time_describe = self.log_eval.df["IMU Time"].describe()
        # vision_time_describe = self.log_eval.df["Vision Time"].describe()
        # dashboard_time_describe = self.log_eval.df["Dashboard Time"].describe()

        # #describe button presses
        # txt_button_click_describe = self.log_eval.df["Text Button Clicks"].describe()
        # stop_stack_button_click_describe = self.log_eval.df["Stop Stack Button Clicks"].describe()
        # change_behavior_view_button_click_describe = self.log_eval.df["Change Behavior View Button Clicks"].describe()
        # behavior_tree_button_click_describe = self.log_eval.df["Behavior Tree Button Clicks"].describe()

        # #describe scrolled pixels
        # amount_scrolled_describe = self.log_eval.df["Amount Scrolled"].describe()
        # behavior_scroll_describe = self.log_eval.df["Behavior Scroll"].describe()
        # motor_scroll_describe = self.log_eval.df["Motors Scroll"].describe()
        # imu_scroll_describe = self.log_eval.df["IMU Scroll"].describe()
        # vision_scroll_describe = self.log_eval.df["Vision Scroll"].describe()
        # dashboard_scroll_describe = self.log_eval.df["Dashboard Scroll"].describe()

        # self.descriptive_log_data = pd.concat([behavior_time_describe.to_frame(name="Behavior Time"),
        #                                       motors_time_describe.to_frame(name="Motors Time"),
        #                                       imu_time_describe.to_frame(name="IMU Time"),
        #                                       vision_time_describe.to_frame(name="Vision Time"),
        #                                       dashboard_time_describe.to_frame(name="Dashboard Time"),
        #                                       txt_button_click_describe.to_frame(name="Text Button Clicks"),
        #                                       stop_stack_button_click_describe.to_frame(name="Stop Stack Button Clicks"),
        #                                       change_behavior_view_button_click_describe.to_frame(name="Change Behavior View Button Clicks"),
        #                                       behavior_tree_button_click_describe.to_frame(name="Behavior Tree Button Clicks"),
        #                                       amount_scrolled_describe.to_frame(name="Amount Scrolled"),
        #                                       behavior_scroll_describe.to_frame(name="Behavior Scroll"),
        #                                       motor_scroll_describe.to_frame(name="Motor Scroll"),
        #                                       imu_scroll_describe.to_frame(name="IMU Scroll"),
        #                                       vision_scroll_describe.to_frame(name="Vision Scroll"),
        #                                       dashboard_scroll_describe.to_frame(name="Dashboard Scroll"),
        #                                       self.descriptive_log_data],
        #                                      axis=1)


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
        ueq = ue.UEQScore(ios.ios_data, study)
        return ueq.ueq_data

    def is_normal_distribution(self, data: pd.Series) -> tuple[bool, float, float]:
        # Copy the data so we can mutate it
        data_copy = data.copy()

        # Make random generator
        rng = np.random.default_rng(1310)

        # Add small noise to the data to avoid issues with identical values
        # The noise should be uniformly distributed between -0.5 and 0.5 
        # so the integers cover the float range evenly
        data_copy += rng.uniform(-0.5, 0.5, size=len(data_copy))

        # Make Shapiro-Wilk test
        stat, p_value = stats.shapiro(data_copy)
        return p_value > 0.05, stat, p_value

    def calculate_significance(self, data1, data2, is_within_subject):
        data1_normal, data1_stat, data1_p = self.is_normal_distribution(data1)
        data2_normal, data2_stat, data2_p = self.is_normal_distribution(data2)

        if data1_normal and data2_normal:
            if is_within_subject:
                stat, p_value = stats.ttest_rel(data1, data2, alternative="greater")
                test_name = "Paired t-test"
            else:
                stat, p_value = stats.ttest_ind(data1, data2, alternative="greater")
                test_name = "Independent t-test"
        else:
            if is_within_subject:
                stat, p_value = stats.wilcoxon(data1, data2, alternative="greater")
                test_name = "Wilcoxon signed-rank test"
            else:
                stat, p_value = stats.mannwhitneyu(data1, data2, alternative="greater")
                test_name = "Mann-Whitney U test"

        return stat, p_value, test_name, data1_normal, data2_normal,data1_p, data2_p


if __name__ == "__main__":
    data_raw = pd.read_csv("quiz_answers.csv", delimiter=";")

    audio_1_condition = pd.read_csv("first_condition_audio.csv", delimiter=";")
    audio_2_condition = pd.read_csv("second_condition_audio.csv", delimiter=";")
    web_1_condition = pd.read_csv("first_condition_web.csv", delimiter=";")
    web_2_condition = pd.read_csv("second_condition_web.csv", delimiter=";")

    #path = "/homes/21wedmann/colcon_ws/src/bitbots_main/bitbots_misc/bitbots_education/scripts/logs"
    #dir_list = os.listdir(path)

    evaluation = StudyEvaluation(audio_1_condition, audio_2_condition, web_1_condition, web_2_condition, data_raw) #dir_list)
    

    evaluation.significance_data.to_csv("significance_data.csv")
    evaluation.descriptive_data.to_csv("descriptive_data.csv")
    #evaluation.descriptive_log_data.to_csv("descriptive_log_data.csv")

    sex_of_audio_group = dm.Demographic(evaluation.quiz_data["Wie viel Kontak.. "], "Wie viel Kontak.. ")
    print(sex_of_audio_group.demographic_data.value_counts())
    #evaluation.log_eval.df.sort_values(by=["VP"], inplace=True)
    #evaluation.log_eval.df.to_csv("evaluation_output.c

    # This code block is used to visualize the quiz scores using a box plot.
    #colors1 = ['lightskyblue', 'darkslateblue', 'lavender', 'mediumpurple' ] #,'plum'
    #colors2 = ['darkslateblue', 'lightskyblue']
    # fig, axs = plt.subplots(figsize=(4,4))
    # bplot1 = axs.boxplot([evaluation.audio1_data['Quiz Score'], evaluation.web1_data['Quiz Score'], 
    #              evaluation.quiz_data['Quiz Score']], patch_artist=True, labels=['Audio', 'Web', 'Baseline'])
    # axs.set_ylabel("Quiz Score")
    # axs.set_title("Between-Subject Quiz Score")
    # axs.set_yticks(range(4, 16, 1))
    # for patch, color in zip(bplot1['boxes'], colors1):
    #     patch.set_facecolor(color)

    # plt.tight_layout()
    # plt.show()

    # fig2, (axs2, axs3) = plt.subplots(1, 2, figsize=(7, 4))
    # fig2.suptitle("Learning Effects for the Quiz Scores")
    # bplot2 = axs2.boxplot([evaluation.audio1_data['Quiz Score'], evaluation.audio2_data['Quiz Score']], 
    #             patch_artist=True, labels=['Audio', 'Web'])
    # axs2.set_title("Audio First Condition")
    # axs3.set_title("Web First Condition")
    # bplot3 = axs3.boxplot([evaluation.web1_data['Quiz Score'], evaluation.web2_data['Quiz Score']],
    #              patch_artist=True, labels=['Web', 'Audio'])
    # axs2.set_ylabel("Quiz Score")
    # axs3.set_ylabel("Quiz Score")
    # for patch, color in zip(bplot2['boxes'], colors1):
    #     patch.set_facecolor(color)
    # for patch, color in zip(bplot3['boxes'], colors2):
    #     patch.set_facecolor(color)
    # plt.tight_layout()
    # plt.show()

    # fig4, axs4 = plt.subplots(figsize=(4,4))
    # bplot4 = axs4.boxplot([pd.concat([evaluation.audio1_data['SUS Score'], evaluation.web2_data['SUS Score']]),
    #               pd.concat([evaluation.web1_data['SUS Score'], evaluation.audio2_data['SUS Score']])], 
    #               patch_artist=True, labels=['Audio', 'Web'])
    # axs4.set_ylabel("SUS Score")
    # axs4.set_title("Within-Subject SUS Score")
    # for patch, color in zip(bplot4['boxes'], colors1):
    #     patch.set_facecolor(color)
    # plt.tight_layout()
    # plt.show()

    # fig5, axs5 = plt.subplots(figsize=(4,4))
    # bplot5 = axs5.boxplot([evaluation.audio1_data['IOS Robot Score'], evaluation.web1_data['IOS Robot Score']],
    #                        patch_artist=True, labels=['Audio', 'Web'])
    # axs5.set_ylabel("IOS Robot Score")
    # axs5.set_title("Between-Subject IOS Robot Score")
    # for patch, color in zip(bplot5['boxes'], colors1):
    #     patch.set_facecolor(color)
    # plt.tight_layout()
    # plt.show()

    # fig6, axs6 = plt.subplots(figsize=(6,4))
    # bplot6 = axs6.boxplot([evaluation.log_eval.df['Behavior Time'], evaluation.log_eval.df['Motors Time'],
    #                        evaluation.log_eval.df['IMU Time'], evaluation.log_eval.df['Vision Time']],
    #                        patch_artist=True, labels=['Behavior', 'Motors', 'IMU', 'Vision'])
    # axs6.set_ylabel("Time Spent (s)")
    # axs6.set_title("Time Spent on Different Pages")
    # for patch, color in zip(bplot6['boxes'], colors1):
    #     patch.set_facecolor(color)
    # plt.tight_layout()
    # plt.show()

    # fig7, axs7 = plt.subplots(figsize=(6,4))
    # bplot7 = axs7.boxplot([evaluation.log_eval.df['Text Button Clicks'], evaluation.log_eval.df['Stop Stack Button Clicks'],
    #                        evaluation.log_eval.df['Change Behavior View Button Clicks'], evaluation.log_eval.df['Behavior Tree Button Clicks']],
    #                        patch_artist=True, labels=['Text', 'Stop Stack', 'Change Behavior View', 'Behavior Tree'])
    # axs7.set_ylabel("Number of Clicks")
    # axs7.set_title("Button Clicks on Different Buttons")
    # for patch, color in zip(bplot7['boxes'], colors1):
    #     patch.set_facecolor(color)
    # plt.tight_layout()
    # plt.show()

    # fig8, axs8 = plt.subplots(figsize=(6,4))
    # bplot8 = axs8.boxplot([evaluation.log_eval.df['Behavior Scroll'], evaluation.log_eval.df['Motors Scroll'],
    #                        evaluation.log_eval.df['IMU Scroll'], evaluation.log_eval.df['Vision Scroll']],
    #                        patch_artist=True, labels=['Behavior', 'Motors', 'IMU', 'Vision',])
    # axs8.set_ylabel("Amount Scrolled (pixels)")
    # axs8.set_title("Amount Scrolled on Different Pages")
    # for patch, color in zip(bplot8['boxes'], colors1):
    #     patch.set_facecolor(color)
    # plt.tight_layout()
    # plt.show()

    # Define positions so boxes appear in pairs
    # positions = [1, 2, 4, 5, 7, 8]  # two per group, with gaps between

    # fig9, axs9 = plt.subplots(figsize=(7,5))
    # bplot9 = axs9.boxplot([evaluation.audio1_data['UEQs Overall Score'], evaluation.web1_data['UEQs Overall Score'],
    #                                 evaluation.audio1_data['UEQs Pragmatic Score'], evaluation.web1_data['UEQs Pragmatic Score'],
    #                                   evaluation.audio1_data['UEQs Hedonic Score'], evaluation.web1_data['UEQs Hedonic Score']],
    #                       patch_artist=True, positions= positions,)
    # axs9.set_yticks(range(1, 10, 1))

    # # Set shared x-axis labels
    # axs9.set_xticks([1.5, 4.5, 7.5])
    # axs9.set_xticklabels(['Overall', 'Pragmatic', 'Hedonic'])

    # # Colors for each pair
    # audio_color = 'lightskyblue'
    # web_color = 'darkslateblue'
    # colors = [audio_color, web_color] * 3

    # # Apply colors
    # for patch, color in zip(bplot9['boxes'], colors):
    #     patch.set_facecolor(color)

    # # Create legend patches manually
    # audio_patch = mpatches.Patch(color=audio_color, label='Audio')
    # web_patch = mpatches.Patch(color=web_color, label='Web')

    # # Add legend to the plot
    # axs9.legend(handles=[audio_patch, web_patch], title='Condition', loc='upper right')

    # axs9.set_title('Within-Subject UEQ Scores')
    # axs9.set_ylabel('UEQ Score')

    # plt.tight_layout()
    # plt.show()

    # fig10, axs10 = plt.subplots(figsize=(7,5))
    # bplot10 = axs10.boxplot([pd.concat([evaluation.audio1_data['UEQs Overall Score'], evaluation.web2_data['UEQs Overall Score']]),
    #                pd.concat([evaluation.web1_data['UEQs Overall Score'], evaluation.audio2_data['UEQs Overall Score']]),
    #                          pd.concat([evaluation.audio1_data['UEQs Pragmatic Score'], evaluation.web2_data['UEQs Pragmatic Score']]),
    #               pd.concat([evaluation.web1_data['UEQs Pragmatic Score'], evaluation.audio2_data['UEQs Pragmatic Score']]),
    #                         pd.concat([evaluation.audio1_data['UEQs Hedonic Score'], evaluation.web2_data['UEQs Hedonic Score']]),
    #              pd.concat([evaluation.web1_data['UEQs Hedonic Score'], evaluation.audio2_data['UEQs Hedonic Score']])],
    #                       patch_artist=True, positions= positions,)
    
    # axs10.set_yticks(range(1, 10, 1))

    # # Set shared x-axis labels
    # axs10.set_xticks([1.5, 4.5, 7.5])
    # axs10.set_xticklabels(['Overall', 'Pragmatic', 'Hedonic'])

    # # Apply colors
    # for patch, color in zip(bplot10['boxes'], colors):
    #     patch.set_facecolor(color)


    # # Add legend to the plot
    # axs10.legend(handles=[audio_patch, web_patch], title='Condition', loc='upper right')

    # axs10.set_title('Between-Subject UEQ Scores')
    # axs10.set_ylabel('UEQ Score')

    # plt.tight_layout()
    # plt.show()

