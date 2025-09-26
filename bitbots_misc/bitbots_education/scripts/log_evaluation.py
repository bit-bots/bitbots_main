from datetime import datetime, timedelta

import pandas as pd


class LogEvaluation:
    def __init__(self, log_data_lists: list ):
        self.data_lits = log_data_lists
        self.button_dict = {}
        self.df = pd.DataFrame(
            {
                "VP": [],
                "Behavior Time": [],
                "Motors Time": [],
                "IMU Time": [],
                "Vision Time": [],
                "Dashboard Time": [],
                "Text Button Clicks": [],
                "Stop Stack Button Clicks": [],
                "Change Behavior View Button Clicks": [],
                "Behavior Tree Button Clicks": [],
                "Amount Scrolled": [],
                "Behavior Scroll": [],
                "Motors Scroll": [],
                "IMU Scroll": [],
                "Vision Scroll": [],
                "Dashboard Scroll": [],
            }

        )
        for data in self.data_lits:
            self.get_page_times(data)

    def get_page_times(self, data):
        #times spent on each page
        behavior_time = timedelta(0)
        motors_time = timedelta(0)
        imu_time = timedelta(0)
        vision_time = timedelta(0)
        dashboard_time = timedelta(0)

        behavior_scroll = 0
        motors_scroll = 0
        imu_scroll = 0
        vision_scroll = 0
        dashboard_scroll = 0

        #number of button clicks ordered by type
        text_button_clicks = 0
        stop_stack_button_clicks = 0
        change_behavior_view_button_clicks = 0
        behavior_tree_button_clicks = 0

        #amount scrolled
        amount_scrolled = 0
        last_scroll_position = 0

        #list to store VP id
        vp_list = [data.iloc[0]["VP"]]

        #lists to store times
        behavior_list = []
        motors_list = []
        imu_list = []
        vision_list = []
        dashboard_list = []

        #lists to store button clicks
        text_button_list = []
        stop_stack_button_list = []
        change_behavior_view_button_list = []
        behavior_tree_button_list = []

        #lists to store amount scrolled
        amount_scrolled_list = []
        behavior_scroll_list = []
        motors_scroll_list = []
        imu_scroll_list = []
        vision_scroll_list = []
        dashboard_scroll_list = []
        
        #initialize current VP, page and start time
        current_vp = data.iloc[0]["VP"]
        current_page = data.iloc[0]["Page"]
        # restart_time = self.find_start_time(self.log_data.iloc[-1]["Timestamp"])
        current_page_start_time: datetime = data.iloc[0]["Timestamp"]

        for _, row in data.iterrows():
            self.count_buttons_in_dict(row)
            # count button clicks
            match row["Status"]:
                case "aufgeklappt" | "eingeklappt":
                    text_button_clicks += 1
                case "gestoppt" | "gestartet":
                    stop_stack_button_clicks += 1
                case "stack" | "tree":
                    change_behavior_view_button_clicks += 1
                case "true" | "false":
                    behavior_tree_button_clicks += 1


            # calculate time spent on each page
            if row["Page"] != current_page and row["VP"] == current_vp:
                last_scroll_position = 0
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

            # if VP changes, store times and clicks, reset counters
            elif row["VP"] != current_vp or row["VP"] == current_vp and row["Timestamp"] == data.iloc[-1]["Timestamp"]:
                last_scroll_position = 0
                # calculate time spent on last page of the previous VP
                time_spent = datetime.fromtimestamp(data["Timestamp"].max()) - datetime.fromtimestamp(
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

                if not row["Timestamp"] == data.iloc[-1]["Timestamp"]:
                    # get times for current VP
                    current_page = row["Page"]
                    current_page_start_time = row["Timestamp"]
                    behavior_list.append(behavior_time.total_seconds())
                    motors_list.append(motors_time.total_seconds())
                    imu_list.append(imu_time.total_seconds())
                    vision_list.append(vision_time.total_seconds())
                    dashboard_list.append(dashboard_time.total_seconds())

                    # get button clicks for current VP
                    text_button_list.append(text_button_clicks)
                    stop_stack_button_list.append(stop_stack_button_clicks)
                    change_behavior_view_button_list.append(change_behavior_view_button_clicks)
                    behavior_tree_button_list.append(behavior_tree_button_clicks)

                    # get amount scrolled for current VP
                    amount_scrolled_list.append(amount_scrolled)
                    behavior_scroll_list.append(behavior_scroll)
                    motors_scroll_list.append(motors_scroll)
                    imu_scroll_list.append(imu_scroll)
                    vision_scroll_list.append(vision_scroll)
                    dashboard_scroll_list.append(dashboard_scroll)

                    # reset times for next VP
                    behavior_time = timedelta(0)
                    motors_time = timedelta(0)
                    imu_time = timedelta(0)
                    vision_time = timedelta(0)
                    dashboard_time = timedelta(0)

                    # reset button clicks for next VP
                    text_button_clicks = 0
                    stop_stack_button_clicks = 0
                    change_behavior_view_button_clicks = 0
                    behavior_tree_button_clicks = 0

                    # reset amount scrolled for next VP
                    amount_scrolled = 0
                    behavior_scroll = 0
                    motors_scroll = 0 
                    imu_scroll = 0
                    vision_scroll = 0
                    dashboard_scroll = 0

            if row["Button"] == "scroll":
                if "scrolledUpTo" in row["Status"]:
                    scrolled = last_scroll_position - int(float(row["Status"].split("scrolledUpTo ")[1]))
                    amount_scrolled += scrolled
                    last_scroll_position = int(float(row["Status"].split("scrolledUpTo ")[1]))
                    match current_page:
                        case "behavior":
                            behavior_scroll += scrolled
                        case "motors":
                            motors_scroll += scrolled
                        case "imu":
                            imu_scroll += scrolled
                        case "vision":
                            vision_scroll += scrolled
                        case "dashboard":
                            dashboard_scroll += scrolled
                        case _:
                            raise ValueError(f"Unknown page name: {current_page}")
                elif "scrolledDownTo" in row["Status"]:
                    scrolled = int(float(row["Status"].split("scrolledDownTo ")[1])) - last_scroll_position
                    amount_scrolled += scrolled
                    last_scroll_position = int(float(row["Status"].split("scrolledDownTo ")[1]))
                    match current_page:
                        case "behavior":
                            behavior_scroll += scrolled
                        case "motors":
                            motors_scroll += scrolled
                        case "imu":
                            imu_scroll += scrolled
                        case "vision":
                            vision_scroll += scrolled
                        case "dashboard":
                            dashboard_scroll += scrolled
                        case _:
                            raise ValueError(f"Unknown page name: {current_page}")


        # Append times for the last VP
        behavior_list.append(behavior_time.total_seconds())
        motors_list.append(motors_time.total_seconds())
        imu_list.append(imu_time.total_seconds())
        vision_list.append(vision_time.total_seconds())
        dashboard_list.append(dashboard_time.total_seconds())

        # Append button clicks for the last VP
        text_button_list.append(text_button_clicks)
        stop_stack_button_list.append(stop_stack_button_clicks)
        change_behavior_view_button_list.append(change_behavior_view_button_clicks)
        behavior_tree_button_list.append(behavior_tree_button_clicks)

        # Append amount scrolled for the last VP
        amount_scrolled_list.append(amount_scrolled)
        behavior_scroll_list.append(behavior_scroll)
        motors_scroll_list.append(motors_scroll)            
        imu_scroll_list.append(imu_scroll)
        vision_scroll_list.append(vision_scroll)
        dashboard_scroll_list.append(dashboard_scroll)

        # Create series for each metric
        # vp series
        vp_series = pd.Series(vp_list, name="VP")

        # time spent series
        behavior_time_series = pd.Series(behavior_list, name="Behavior Time")
        motors_time_series = pd.Series(motors_list, name="Motors Time")
        imu_time_series = pd.Series(imu_list, name="IMU Time")
        vision_time_series = pd.Series(vision_list, name="Vision Time")
        dashboard_time_series = pd.Series(dashboard_list, name="Dashboard Time")

        # button clicks series
        text_button_clicks_series = pd.Series(text_button_list, name="Text Button Clicks")
        stop_stack_button_clicks_series = pd.Series(stop_stack_button_list, name="Stop Stack Button Clicks")
        change_behavior_view_button_clicks_series = pd.Series(change_behavior_view_button_list, name="Change Behavior View Button Clicks")
        behavior_tree_button_clicks_series = pd.Series(behavior_tree_button_list, name="Behavior Tree Button Clicks")

        # amount scrolled series
        amount_scrolled_series = pd.Series(amount_scrolled_list, name="Amount Scrolled")
        behavior_scroll_series = pd.Series(behavior_scroll_list, name="Behavior Scroll")
        motors_scroll_series = pd.Series(motors_scroll_list, name="Motors Scroll")
        imu_scroll_series = pd.Series(imu_scroll_list, name="IMU Scroll")
        vision_scroll_series = pd.Series(vision_scroll_list, name="Vision Scroll")
        dashboard_scroll_series = pd.Series(dashboard_scroll_list, name="Dashboard Scroll")

        # concatenate all series into a single DataFrame
        current_data_frame = pd.concat(
            [
                vp_series,
                behavior_time_series,
                motors_time_series,
                imu_time_series,
                vision_time_series,
                dashboard_time_series,
                text_button_clicks_series,
                stop_stack_button_clicks_series,
                change_behavior_view_button_clicks_series,
                behavior_tree_button_clicks_series,
                amount_scrolled_series,
                behavior_scroll_series,
                motors_scroll_series,
                imu_scroll_series,
                vision_scroll_series,
                dashboard_scroll_series,
            ],
            axis=1,
        )
        self.df = pd.concat([self.df, current_data_frame])
    # def find_start_time(self, time):
    #     time = datetime.fromtimestamp(time)
    #     duration = timedelta(minutes=6, seconds=30)
    #     new_time = time - duration
    #     return new_time.timestamp()

    def count_buttons_in_dict(self, row):
        if (
            row["Status"] == "aufgeklappt"
            or row["Status"] == "eingeklappt"
            or row["Status"] == "gestoppt"
            or row["Status"] == "gestartet"
            or row["Status"] == "stack"
            or row["Status"] == "tree"
            or row["Status"] == "true"
            or row["Status"] == "false"
        ):
            button_name = row["Button"] + " " + row["Page"]
            button_status = row["Status"]

            if button_name not in self.button_dict:
                self.button_dict[button_name] = {}

            if button_status not in self.button_dict[button_name]:
                self.button_dict[button_name][button_status] = 0

            self.button_dict[button_name][button_status] += 1


# data_raw1 = pd.read_csv("04.09.2025.csv")
# data_raw2 = pd.read_csv("04.09.2025(2).csv")
# data_sorted1 = data_raw1.sort_values(by=["VP", "Timestamp"])
# data_sorted1["Page"].fillna("dashboard", inplace=True)
# data_sorted2 = data_raw2.sort_values(by=["VP", "Timestamp"])
# data_sorted2["Page"].fillna("dashboard", inplace=True)
# eval = LogEvaluation([data_sorted1, data_sorted2])
# print(eval.button_dict, eval.df)
