import rospy

from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.msg import Path
from transforms3d.euler import euler2quat


rospy.init_node("tt_b")
sp = rospy.ServiceProxy('move_base/NavfnROS/make_plan', GetPlan)
print("waiting for service....")
sp.wait_for_service()
pub = rospy.Publisher("fake_plan", Path, queue_size=1)

def save_theta_to_request(rq, theta, is_goal=True):
    quat = euler2quat(0,0,theta)
    if is_goal:
        rq.goal.pose.orientation.w = quat[0]
        rq.goal.pose.orientation.x = quat[1]
        rq.goal.pose.orientation.y = quat[2]
        rq.goal.pose.orientation.z = quat[3]
    else:
        rq.start.pose.orientation.w = quat[0]
        rq.start.pose.orientation.x = quat[1]
        rq.start.pose.orientation.y = quat[2]
        rq.start.pose.orientation.z = quat[3]


def send_goal_and_publish(start_x,start_y,start_theta, goal_x,goal_y,goal_theta,):
    rq = GetPlanRequest()
    rq.start.header.frame_id = "map"
    rq.goal.header.frame_id = "map"
    rq.start.pose.position.x = start_x
    rq.start.pose.position.y = start_y
    save_theta_to_request(rq, start_theta, is_goal=False)
    rq.goal.pose.position.x = goal_x
    rq.goal.pose.position.y = goal_y
    save_theta_to_request(rq, start_theta, is_goal=True)
    res = sp(rq)
    for i in range(10):
        pub.publish(res.plan)
        rospy.sleep(0.1)
