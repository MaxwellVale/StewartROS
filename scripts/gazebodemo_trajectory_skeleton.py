#!/usr/bin/env python3
#
#   gazebodemo_trajectory.py
#
#   Create a motion, to send to Gazebo for the sevenDOF.
#
#   Publish:   /sevendof/j1_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j2_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j3_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j4_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j5_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j6_pd_control/command    std_msgs/Float64
#   Publish:   /sevendof/j7_pd_control/command    std_msgs/Float64
#
import rospy
import numpy as np

from sensor_msgs.msg     import JointState
from std_msgs.msg        import Float64
from urdf_parser_py.urdf import Robot

# Import the kinematics stuff:
from kinematics import Kinematics, p_from_T, R_from_T, Rx, Ry, Rz
# We could also import the whole thing ("import kinematics"),
# but then we'd have to write "kinematics.p_from_T()" ...

# Import the Spline stuff:
from splines import  CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5


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

        # Find the simulation's starting position.  This will block,
        # but that's appropriate if we don't want to start until we
        # have this information.  Of course, the simulation starts at
        # zero, so we can simply use that information too.
        msg = rospy.wait_for_message('/sevendof/joint_states', JointState);
        theta0 = np.array(msg.position).reshape((-1,1))
        rospy.loginfo("Gazebo's starting position: %s", str(theta0.T))

        # IF we wanted to do kinematics:
        # # Grab the robot's URDF from the parameter server.
        # robot = Robot.from_parameter_server()
        # # Instantiate the Kinematics
        # self.kin = Kinematics(robot, 'world', 'tip')

        # Pick a starting and final joint position.
        thetaA = np.zeros((self.N, 1))
        thetaB = np.array([-np.pi/2, np.pi/4, 0.0, -np.pi/2, 0.0, -np.pi/4, 0.0]).reshape((-1,1))

        # Create the trajectory segments.  When the simulation first
        # turns on, the robot sags slightly due to it own weight.  So
        # we start with a 2s hold to allow any ringing to die out.
        self.segments = (Hold(thetaA, 2.0),)

        FIX THIS: DO YOU WANT MORE SEGMENTS?

        # Also reset the trajectory, starting at the beginning.
        self.reset()

    # Reset.  If the simulation resets, also restart the trajectory.
    def reset(self):
        # Just reset the segment index counter and start time to zero.
        self.t0    = 0.0
        self.index = 0


    # Update is called every 10ms of simulation time!
    def update(self, t, dt):
        # If the current trajectory segment is done, shift to the next.
        dur = self.segments[self.index].duration()
        if (t - self.t0 >= dur):
            self.t0    = (self.t0    + dur)
            #self.index = (self.index + 1)                       # not cyclic!
            self.index = (self.index + 1) % len(self.segments)  # cyclic!

        # Check whether we are done with all trajectory segments.
        if (self.index >= len(self.segments)):
            rospy.signal_shutdown("Done with motion")
            return


        # Grab the spline output as joint values.
        (theta, thetadot) = self.segments[self.index].evaluate(t - self.t0)

        FIX THIS: OR YOU MAY DECIDE TO PROGRAM SOME OTHER FORM OF TRAJECTORY?


        # Send the individal angle commands.
        for i in range(self.N):
            self.pubs[i].publish(Float64(theta[i]))


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('trajectory')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).  This
    # relies on rospy.Time, which is set by the simulation.  Therefore
    # slower-than-realtime simulations propagate correctly.
    starttime = rospy.Time.now()
    lasttime  = starttime
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t  = (servotime - starttime).to_sec()
        dt = (servotime - lasttime).to_sec()
        lasttime = servotime

        # Update the controller.
        generator.update(t, dt)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.  Note, if you reset the
        # simulation, the time jumps back to zero and triggers an
        # exception.  If desired, we can simple reset the time here to
        # and start all over again.
        try:
            servo.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # Reset the time counters, as well as the trajectory
            # generator object.
            rospy.loginfo("Resetting...")
            generator.reset()
            starttime = rospy.Time.now()
            lasttime  = starttime
            
            
