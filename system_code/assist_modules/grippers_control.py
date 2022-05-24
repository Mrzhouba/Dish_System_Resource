import os
import rospy
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal

class grippers_control:
    def AG95_gripper(self,AG95_Force,AG95_Position, action_client):
        x = 1
        y = AG95_Force
        z = AG95_Position
        # print x,y,z
        goal = ActuateHandGoal(x,y,z)
        action_client.send_goal(goal)
        action_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def AG95_gripper_delay(self, AG95_Force, AG95_Position, action_client):
        rospy.sleep(1.1)
        x = 1
        y = AG95_Force
        z = AG95_Position
        goal = ActuateHandGoal(x, y, z)
        action_client.send_goal(goal)
        action_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def Airpick_gripper(self,x):
        if (x == 1):
            os.system('rosservice call /ur3/ur_driver/set_io "fun: 1\npin: 2\nstate: 1.0"')
        else:
            os.system('rosservice call /ur3/ur_driver/set_io "fun: 1\npin: 2\nstate: 0.0"')
