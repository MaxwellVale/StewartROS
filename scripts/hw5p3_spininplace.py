#!/usr/bin/env python3
#
#   HW#5 P3 spininplace.py
#
#   Visualize the 6DOF, rotating the cube in place.
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import numpy as np

from sensor_msgs.msg     import JointState
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
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        rospy.sleep(0.25)

        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = Kinematics(robot, 'world', 'tip')

        # Create the splines for the path variable.
        self.segments = (Hold(0.0,      1.0, 'Path'),
                         Goto(0.0, 1.0, 2.0, 'Path'),
                         Goto(1.0, 0.0, 2.0, 'Path'))

        # Initialize the current segment index and starting time t0.
        self.index = 0
        self.t0    = 0.0

        # Initialize the storage of the last joint values (column vector).
        self.lasttheta = \
            np.array([-0.2, 0.7, -1.6, 1.4, 0.5, -1.5]).reshape((-1,1))

        # Initialize/save the parameter.
        self.lam = 20

    # Path
    def pd(self, s):
        return np.array([0.0, 0.6, 0.5]).reshape((3,1))
    def Rd(self, s):
        return Rz(-np.pi * s) @ Ry(- np.pi/2)
    def vd(self, s, sdot):
        return np.array([0.0, 0.0, 0.0]).reshape((3,1)) * sdot
    def wd(self, s, sdot):
        return np.array([0.0, 0.0, -np.pi]).reshape((3,1)) * sdot

    # Error functions
    def ep(self, pd, pa):
        return (pd - pa)
    def eR(self, Rd, Ra):
        return 0.5*(np.cross(Ra[:,0:1], Rd[:,0:1], axis=0) +
                    np.cross(Ra[:,1:2], Rd[:,1:2], axis=0) +
                    np.cross(Ra[:,2:3], Rd[:,2:3], axis=0))

    # Update is called every 10ms!
    def update(self, t, dt):
        # If the current segment is done, shift to the next.
        dur = self.segments[self.index].duration()
        if (t - self.t0 >= dur):
            self.t0    = (self.t0    + dur)
            self.index = (self.index + 1)                       # not cyclic!
            #self.index = (self.index + 1) % len(self.segments)  # cyclic!

        # Check whether we are done with all segments.
        if (self.index >= len(self.segments)):
            rospy.signal_shutdown("Done with motion")
            return


        # Determine the desired tip position/rotation/velocities (task
        # information) for the current time.  Start grabbing the
        # current path variable (from the spline segment).  Then use
        # the above functions to convert to p/R/v/w:
        (s, sdot) = self.segments[self.index].evaluate(t - self.t0)
        pd = self.pd(s)
        Rd = self.Rd(s)
        vd = self.vd(s,sdot)
        wd = self.wd(s,sdot)


        # Then start at the last cycle's joint values.
        theta = self.lasttheta

        # Compute the forward kinematics (using last cycle's theta),
        # extracting the position and orientation.
        (T,J) = self.kin.fkin(theta)
        p     = p_from_T(T)
        R     = R_from_T(T)

        # Stack the linear and rotation reference velocities (summing
        # the desired velocities and scaled errors)
        xrdot = np.vstack((vd + self.lam * self.ep(pd, p),
                           wd + self.lam * self.eR(Rd, R)))

        # Take an IK step, using Euler integration to advance the joints.
        thetadot = np.linalg.pinv(J) @ xrdot
        theta    = theta + dt * thetadot


        # Save the joint values (to be used next cycle).
        self.lasttheta = theta

        # Collect and send the JointState message (with the current time).
        cmdmsg = JointState()
        cmdmsg.name         = ['theta1', 'theta2', 'theta3',
                               'theta4', 'theta5', 'theta6']
        cmdmsg.position     = theta
        cmdmsg.velocity     = thetadot
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('spininplace')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    t = 0
    while not rospy.is_shutdown():

        # Update the controller.
        generator.update(t, dt)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()

        # Update the time.
        t += dt
