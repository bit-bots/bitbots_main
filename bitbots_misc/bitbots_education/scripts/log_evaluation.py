from datetime import datetime, timedelta

import pandas as pd


class LogEvaluation:
    def __init__(self, log_data1, log_data2):
        self.log_data = log_data1
        self.log_data2 = log_data2
        self.button_dict = {}
        self.df = pd.DataFrame(
            {"VP": [], "Behavior Time": [], "Motors Time": [], "IMU Time": [], "Vision Time": [], "Dashboard Time": []}
        )
        self.get_page_times(log_data1)
        self.get_page_times(log_data2)

    def get_page_times(self, data):
        behavior_time = timedelta(0)
        motors_time = timedelta(0)
        imu_time = timedelta(0)
        vision_time = timedelta(0)
        dashboard_time = timedelta(0)

        vp_list = [data.iloc[0]["VP"]]
        behavior_list = []
        motors_list = []
        imu_list = []
        vision_list = []
        dashboard_list = []

        current_vp = data.iloc[0]["VP"]
        current_page = data.iloc[0]["Page"]
        # restart_time = self.find_start_time(self.log_data.iloc[-1]["Timestamp"])
        current_page_start_time: datetime = data.iloc[0]["Timestamp"]

        for _, row in data.iterrows():
            self.count_buttons(row)
            if row["Page"] != current_page and row["VP"] == current_vp:
                time_spent = datetime.fromtimestamp(row["Timestamp"]) - datetime.fromtimestamp(current_page_start_time)
                match current_page:
                    case "behavior":
                        behavior_time += time_spent
                    case "motors":
                        motors_time += time_spent
                    case "imu":
                        imu_time += time_spent
                    case "vision":
                        vision_time += time_spent
                    case "dashboard":
                        dashboard_time += time_spent
                    case _:
                        raise ValueError(f"Unknown page name: {current_page}")
                current_page = row["Page"]
                current_page_start_time = row["Timestamp"]

            elif row["VP"] != current_vp:
                time_spent = datetime.fromtimestamp(data.iloc[-1]["Timestamp"]) - datetime.fromtimestamp(
                    current_page_start_time
                )
                match current_page:
                    case "behavior":
                        behavior_time += time_spent
                    case "motors":
                        motors_time += time_spent
                    case "imu":
                        imu_time += time_spent
                    case "vision":
                        vision_time += time_spent
                    case "dashboard":
                        dashboard_time += time_spent
                    case _:
                        raise ValueError(f"Unknown page name: {current_page}")
                current_vp = row["VP"]
                if current_vp not in vp_list:
                    vp_list.append(current_vp)

                # get times for first VP
                current_page = row["Page"]
                current_page_start_time = row["Timestamp"]
                behavior_list.append(behavior_time)
                motors_list.append(motors_time)
                imu_list.append(imu_time)
                vision_list.append(vision_time)
                dashboard_list.append(dashboard_time)

                # reset times for next VP
                behavior_time = timedelta(0)
                motors_time = timedelta(0)
                imu_time = timedelta(0)
                vision_time = timedelta(0)
                dashboard_time = timedelta(0)

        # Append times for the last VP
        behavior_list.append(behavior_time)
        motors_list.append(motors_time)
        imu_list.append(imu_time)
        vision_list.append(vision_time)
        dashboard_list.append(dashboard_time)

        vp_series = pd.Series(vp_list, name="VP")
        behavior_time_series = pd.Series(behavior_list, name="Behavior Time")
        motors_time_series = pd.Series(motors_list, name="Motors Time")
        imu_time_series = pd.Series(imu_list, name="IMU Time")
        vision_time_series = pd.Series(vision_list, name="Vision Time")
        dashboard_time_series = pd.Series(dashboard_list, name="Dashboard Time")
        current_data_frame = pd.concat(
            [
                vp_series,
                behavior_time_series,
                motors_time_series,
                imu_time_series,
                vision_time_series,
                dashboard_time_series,
            ],
            axis=1,
        )
        self.df = pd.concat([self.df, current_data_frame])

    # def find_start_time(self, time):
    #     time = datetime.fromtimestamp(time)
    #     duration = timedelta(minutes=6, seconds=30)
    #     new_time = time - duration
    #     return new_time.timestamp()

    def count_buttons(self, row):
        if (
            row["Status"] == "aufgeklappt"
            or row["Status"] == "eingeklappt"
            or row["Status"] == "gestoppt"
            or row["Status"] == "gestartet"
            or row["Status"] == "stack"
            or row["Status"] == "tree"
        ):
            button_name = row["Button"] + " " + row["Page"]
            button_status = row["Status"]

            if button_name not in self.button_dict:
                self.button_dict[button_name] = {}

            if button_status not in self.button_dict[button_name]:
                self.button_dict[button_name][button_status] = 0

            self.button_dict[button_name][button_status] += 1


data_raw1 = pd.read_csv("04.09.2025.csv")
data_raw2 = pd.read_csv("04.09.2025(2).csv")
data_sorted1 = data_raw1.sort_values(by=["VP", "Timestamp"])
data_sorted1["Page"].fillna("dashboard", inplace=True)
data_sorted2 = data_raw2.sort_values(by=["VP", "Timestamp"])
data_sorted2["Page"].fillna("dashboard", inplace=True)
eval = LogEvaluation(data_sorted1, data_sorted2)
print(eval.button_dict)
