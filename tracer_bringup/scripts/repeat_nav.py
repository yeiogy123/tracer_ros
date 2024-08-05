import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

class MoveClient():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
    
    def move(self, x=0.5, y=0.0, forward:bool=True):
        self.client.cancel_all_goals()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        if forward:
            goal.target_pose.pose.orientation.w = 1.0
        else:
            goal.target_pose.pose.orientation.z = 1.0

        rospy.loginfo(f"Goal Command: \n{goal}")

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            return self.client.get_result()
    

def pub_init(pose_pub):
    pos_msg = PoseWithCovarianceStamped()

    pos_msg.header.stamp = rospy.Time.now()
    pos_msg.header.frame_id = "base_link"

    pos_msg.pose.pose.position.x = 2.67
    pos_msg.pose.pose.position.y = 0
    pos_msg.pose.pose.position.z = 0

    pos_msg.pose.pose.orientation.x = 0
    pos_msg.pose.pose.orientation.y = 0
    pos_msg.pose.pose.orientation.z = 0
    pos_msg.pose.pose.orientation.w = 1
    pos_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    
    
    pose_pub.publish(pos_msg)
    

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    pose_pub = rospy.Publisher("/rtabmap/initialpose", PoseWithCovarianceStamped, queue_size=5)
    # nav_stack = [[36.7, -4.7],
    #              [0., 0.]]
    
    # 11F 017
    # nav_stack = [[19.2, -2.7, True],
    #              [34.7, -4.4, True],
    #              [19.2, -2.7, False],
    #              [0., 0., True]]
    
    # 9F New
    nav_stack = [#[10.6, -0.3, True],
                 [26.7, -0.8, True],
                 #[10.6, -0.3, False],
                 [0., 0., True]]
    
    # 9F New night
    # nav_stack = [#[13.3, -0.87, True],
    #               [26.3, -1.9, True],
    #              #[13.3, -0.87, False],
    #              [0., 0., True]]
    rospy.sleep(3)
    rospy.loginfo("publish initial pose")
    pub_init(pose_pub)
    rospy.sleep(1)
    rospy.loginfo("start navigation")
    moveClient = MoveClient()
    for x, y, forward in nav_stack:
        try:
            result = moveClient.move(x, y, forward)
            # result = movebase_client(x=x, y=y, forward=forward)
            if result:
                rospy.loginfo(f"Reached goal: ({x}, {y})")
            else:
                rospy.loginfo(f"Failed to reach goal: ({x}, {y})")
                break
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")