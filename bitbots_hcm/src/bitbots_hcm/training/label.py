import pickle
import random
import rospy
import keyboard

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu, JointState

from bitbots_hcm.training.dataset import Dataset, Frame
from bitbots_ros_patches.rate import Rate

imu = None
joint_states = None
cop_left = None
cop_right = None

rospy.init_node("labeling")


def imu_cb(msg):
    global imu
    imu = msg


def joint_state_cb(msg):
    global joint_states
    joint_states = msg


def cop_left_cb(msg):
    global cop_left
    cop_left = msg


def cop_right_cb(msg):
    global cop_right
    cop_right = msg


dataset = Dataset()

rospy.Subscriber("/imu/data", Imu, imu_cb, tcp_nodelay=True)
rospy.Subscriber("/joint_states", JointState, joint_state_cb, tcp_nodelay=True)
rospy.Subscriber("/cop_l", PointStamped, cop_left_cb, tcp_nodelay=True)
rospy.Subscriber("/cop_r", PointStamped, cop_right_cb, tcp_nodelay=True)


# set keys
def key_cb(key):
    global current_key
    current_key = key


current_key = '5'
previous_key = None
keyboard.add_hotkey('5', key_cb, args='5')
keyboard.add_hotkey('0', key_cb, args='0')
keyboard.add_hotkey('1', key_cb, args='1')
keyboard.add_hotkey('2', key_cb, args='2')
keyboard.add_hotkey('3', key_cb, args='3')
keyboard.add_hotkey('4', key_cb, args='4')
keyboard.add_hotkey('s', key_cb, args='s')

frequency = 200
rate = Rate(frequency)
print("Press \'s\' top stop. 0,1,2,3,4 to label stable,front,back,left,right")
while True:
    frame = Frame(rospy.Time.now(), joint_states=joint_states, imu=imu, cop_l=cop_left, cop_r=cop_right, image=None)
    if current_key == '5':
        if previous_key != '5':
            print("Not labeling")
            previous_key = current_key
        continue
    elif current_key == '0':
        if previous_key != '0':
            print("Stable")
            previous_key = current_key
        frame.label = 0
    elif current_key == '1':
        if previous_key != '1':
            print("Front")
            previous_key = current_key
        frame.label = 1
    elif current_key == '2':
        if previous_key != '2':
            print("Back")
            previous_key = current_key
        frame.label = 2
    elif current_key == '3':
        if previous_key != '3':
            print("Left")
            previous_key = current_key
        frame.label = 3
    elif current_key == '4':
        if previous_key != '4':
            print("Right")
            previous_key = current_key
        frame.label = 4
    elif current_key == 's':
        break
    else:
        print("key not known")
        continue
    dataset.frames.append(frame)
    rate.sleep()

print(F"Recorded {len(dataset.frames)} frames")
sorted = [0, 0, 0, 0, 0]
for i in range(len(dataset.frames)):
    sorted[dataset.frames[i].label] += 1
print(F'Stable {sorted[0]} Front {sorted[1]} Back {sorted[2]} Left {sorted[3]} Right {sorted[4]}')

time_to_remove = input("Remove how many seconds: ")
try:
    frames_to_remove = int(float(time_to_remove) * frequency)
    print(F"{frames_to_remove} ---")
    dataset.frames = dataset.frames[:-frames_to_remove - 1]
    print(F"Frames after removing: {len(dataset.frames)}")
except:
    pass

reduce_to = input("Reduce number to ")
if reduce_to != "":
    dataset.frames = random.choices(dataset.frames, k=int(reduce_to))
    print(F"Reduced number of frames to {len(dataset.frames)}")

file_name = input("File name: ")
if len(file_name) > 0:
    with open(file_name + ".pkl", "wb") as file:
        pickle.dump(dataset, file)

    print(F"Saved dataset with {len(dataset.frames)} frames to {file_name}.pkl")
