#!/usr/bin/env python3
#
#   gazebodemo_usegui.py
#
#   Watch the sliders and send to Gazebo for the sevenDOF.
#
#   Subscribe: /joint_states                      sensor_msgs/JointState
#   Publish:   /sevendof/j1_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j2_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j3_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j4_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j5_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j6_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j7_pd_control/command    std_msgs/Float64
#
import rospy

from sensor_msgs.msg     import JointState
from std_msgs.msg        import Float64


#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # The Gazebo controllers treat each joint seperately.  We thus
        # need a seperate publisher for each joint under the topic
        # "/BOTNAME/CONTROLLER/command"...
        self.N    = 7
        self.pubs = []
        for i in range(self.N):
            topic = "/sevendof/j" + str(i+1) + "_pd_control/command"
            self.pubs.append(rospy.Publisher(topic, Float64, queue_size=10))

        # # We used to add a short delay to allow the connection to form
        # # before we start sending anything.  However, if we start
        # # Gazebo "paused", this already provides time for the system
        # # to set up, before the clock starts.
        # rospy.sleep(0.25)

        # Create a subscriber to listen to the info coming from the GUI.
        rospy.Subscriber("/joint_states", JointState, self.jointstatemsg)


    # Receive/process a new joint state message.
    def jointstatemsg(self, msg):
        # Take the position data from the incoming messages and
        # re-publish into the seperate topics to Gazebo.
        for i in range(self.N):
            self.pubs[i].publish(Float64(msg.position[i]))


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('trajectory')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Run until shutdown (killed or ctrl-C'ed).  Note the relay action
    # (from GUI to Gazebo) is triggered when a new message arrives
    # from the GUI.  Hence we do not need a servo loop and can just
    # "spin" here (which allows sepereate threads to listen for
    # incoming messages).
    rospy.loginfo("Running...")
    rospy.spin()
