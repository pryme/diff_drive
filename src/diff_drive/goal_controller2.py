from __future__ import division
from math import pi, sqrt, sin, cos, atan2, copysign
from diff_drive.pose import Pose
import rospy  # TODO diagnostic - maybe now reqd
import tf  # TODO maybe only need tf.transformations?

class GoalController:
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.kP = 3
        self.kA = 8
        self.kB = 1.5
        self.maxLinearSpeed = 1E9
        self.maxAngularSpeed = 1E9
        self.maxLinearAcceleration = 1E9
        self.linearTolerance = 0.025 # 2.5cm
        self.angularTolerance = 3/180*pi # 3 degrees
        self.defaultLinearSpeed = 0.2  # m/s; should make this parameter
        self.defaultAngularSpeed = 1.0  # rad/s; should make this parameter

    def setConstants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def setMaxLinearSpeed(self, speed):
        self.maxLinearSpeed = speed

    def setMaxAngularSpeed(self, speed):
        self.maxAngularSpeed = speed

    def setMaxLinearAcceleration(self, accel):
        self.maxLinearAcceleration = accel

    def setLinearTolerance(self, tolerance):
        self.linearTolerance = tolerance

    def setAngularTolerance(self, tolerance):
        self.angularTolerance = tolerance

    def getGoalDistance(self, cur, goal):
        """
        Returns distance (m) from current position to goal position.
        Arguments are now:
          * cur: current pose as geometryMsgs/Pose in odom frame
          * goal: goal pose as geometryMsgs/Pose in odom frame
        """
        if goal is None:
            return 0
        diffX = cur.position.x - goal.position.x
        diffY = cur.position.y - goal.position.y
        return sqrt(diffX*diffX + diffY*diffY)

    def atGoal(self, cur, goal, delta):
        """
        Returns True if we have reached the goal position within tolerances.
        Arguments are now:
          * cur: current pose as geometryMsgs/Pose in odom frame
          * goal: goal pose as geometryMsgs/Pose in odom frame
          * delta: delta pose as geometryMsgs/Pose (only orientation currenlty used)
        """
        if goal is None:
            return True
        d = self.getGoalDistance(cur, goal)
        #dTh = abs(cur.theta - goal.theta)
        # rospy.loginfo("ang_tol: %2.3f; dTh: %2.3f; lin_tol: %2.3f; dist: %2.3f", self.angularTolerance, dTh, self.linearTolerance, d)  # TODO diagnostic        
        
        # is heading close enough?
        delta_numpy_quat = [delta.orientation.x, delta.orientation.y,
            delta.orientation.z, delta.orientation.w]
        delta_euler = tf.transformations.euler_from_quaternion(delta_numpy_quat)
        print "atGoal::delta_euler: (%.3f, %.3f, %.3f)" % (delta_euler[0], delta_euler[1], delta_euler[2])
        
        return d < self.linearTolerance and delta_euler[2] < self.angularTolerance
        
    def getVelocity(self, cur, goal, dT, case):  # adding case arg
    #def getVelocity(self, cur, goal, dT):
        """
        Returns Twist velocities to drive with.
        Arguments are now:
          * cur: current pose as geometryMsgs/Pose in odom frame
          * goal: goal pose as geometryMsgs/Pose in odom frame
          
        Note `desired` is a hybrid Pose struct from Pose.py in this pkg
        """
        desired = Pose()
        d = self.getGoalDistance(cur, goal)
        # TODO oh crap...all the cur.theta no longer exist (and similar)
        current_theta = tf.transformations.euler_from_quaternion([cur.orientation.x,
            cur.orientation.y, cur.orientation.z, cur.orientation.w])[2]
        # print "getVelocity::current_theta: %.3f rad" % current_theta
        a = atan2(goal.position.y - cur.position.y, goal.position.x - cur.position.x) - current_theta
        #b = cur.theta + a - goal.theta
        goal_theta = tf.transformations.euler_from_quaternion([goal.orientation.x,
            goal.orientation.y, goal.orientation.z, goal.orientation.w])[2]
        print "getVelocity::goal_theta: %.3f rad; current_theta: %.3f" % (goal_theta, current_theta)
        b = -a - (current_theta - goal_theta)  # reversing sign change for trial
        
        if case == "backup":
            desired.thetaVel = 0
            if d < self.linearTolerance:
                desired.xVel = 0
            else:
                desired.xVel = -self.defaultLinearSpeed
        elif case == "spin":
            desired.xVel = 0
            dTh = current_theta - goal_theta
            if abs(dTh) < self.angularTolerance:
                desired.thetaVel = 0
            else:
                desired.thetaVel = -copysign(self.defaultAngularSpeed, dTh)
        elif case == "forward":
            if abs(d) < self.linearTolerance:
                desired.xVel = 0
                desired.thetaVel = -self.kB * b
                #desired.thetaVel = self.kB * b  # remove minus sign
            else:
                desired.xVel = self.kP * d
                #desired.thetaVel = self.kA*a + self.kB*b
                desired.thetaVel = self.kA*a - self.kB*b  # sign error on b?
        else:  # this case s/b redundant - use for debugging
            rospy.logwarn("goal_controller.py: invalid case")

        # Adjust velocities if linear or angular rates or accel too high.
        # TBD TODO
        
        return desired
