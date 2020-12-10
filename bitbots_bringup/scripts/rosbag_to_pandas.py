#!/usr/bin/env python3

import rosbag
import argparse
import rosmsg
import pandas as pd

"""
This script reads in a rosbag and ouputs a pandas dataframe representration of the data. This is usefull for later 
processing, e.g. in scikit-learn or for creating plots.
"""


class COLORS:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


parser = argparse.ArgumentParser()
parser.add_argument("bag")

args = parser.parse_args()

bag = rosbag.Bag(args.bag)
topics = []
types = []
for key, value in bag.get_type_and_topic_info().topics.items():
    topics.append(key)
    types.append(value[0])

col_width = max(len(word) for row in [topics, types] for word in row) + 4  #
for i in range(len(topics)):
    print(f"{COLORS.BOLD}{COLORS.OKBLUE}[{i}]:{COLORS.ENDC} {topics[i].ljust(col_width)}{types[i]}")

topic_selections_str = input("Select data source:")

topic_selections_str_list = topic_selections_str.split()
topic_selections = []

for topic_selection_str in topic_selections_str_list:
    try:
        topic_selections.append(int(topic_selection_str))
    except ValueError as ex:
        print(ex)
        exit(-1)

selected_topics_list = [topics[i] for i in topic_selections]

data_selections = []
for topic_selection in topic_selections:
    print(f"{COLORS.BOLD}{COLORS.OKGREEN}Available Fields from {COLORS.OKBLUE}{topics[topic_selection]}{COLORS.ENDC}")
    print(rosmsg.get_msg_text(types[topic_selection]))
    single_topic_data_selection = input()
    data_selections.append(single_topic_data_selection.split())


def recursive_getattr(obj, field_list):
    if len(field_list) == 1:
        return getattr(obj, field_list[0])
    else:
        return recursive_getattr(getattr(obj, field_list[0]), field_list[1:])


frames = []
for data_selection in data_selections:
    frames.append(pd.DataFrame(columns=["header"].extend(data_selection)))

msg_generator = bag.read_messages(topics=[topics[i] for i in topic_selections])

print(f"{COLORS.BOLD}{COLORS.WARNING}Extracting data{COLORS.ENDC}")
for msg in msg_generator:
    try:
        i = selected_topics_list.index(msg.topic)
    except ValueError:
        continue

    fields = {}
    fields["header"] = msg.message.header.stamp.to_sec()
    for data_selection in data_selections[i]:
        fields[data_selection] = recursive_getattr(msg.message, data_selection.split("."))
    frames[i] = frames[i].append(fields, ignore_index=True)

for i, frame in enumerate(frames):
    print(f"{COLORS.OKBLUE}{selected_topics_list[i]}{COLORS.ENDC}")
    print(frame.info())
    frame.to_pickle(selected_topics_list[i][1:].replace("/", "-") + ".pickle")
