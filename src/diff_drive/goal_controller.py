from __future__ import division
from math import pi, sqrt, sin, cos, atan2, copysign
from diff_drive.pose import Pose

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
        self.minLinearSpeed = 0
        self.minAngularSpeed = 0
        self.deadbandLinearSpeed = self.minLinearSpeed / 10.0
        self.deadbandAngularSpeed = self.minAngularSpeed / 10.0
        self.maxLinearAcceleration = 1E9
        self.linearTolerance = 0.025 # 2.5cm
        self.angularTolerance = 3/180*pi # 3 degrees

    def setConstants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def setMaxLinearSpeed(self, speed):
        self.maxLinearSpeed = speed
    
    def setMinLinearSpeed(self, speed):
        self.minLinearSpeed = speed
        self.deadbandLinearSpeed = speed / 10.0

    def setMaxAngularSpeed(self, speed):
        self.maxAngularSpeed = speed
        
    def setMinAngularSpeed(self, speed):
        self.minAngularSpeed = speed
        self.deadbandAngularSpeed = speed / 10.0

    def setMaxLinearAcceleration(self, accel):
        self.maxLinearAcceleration = accel

    def setLinearTolerance(self, tolerance):
        self.linearTolerance = tolerance

    def setAngularTolerance(self, tolerance):
        self.angularTolerance = tolerance

    def getGoalDistance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return sqrt(diffX*diffX + diffY*diffY)

    def atGoal(self, cur, goal, fwd):
        """Decides if goal has been reached.
        
        Args:
            cur: Current pose (2D) from odometry in /odom frame.
            goal: Goal pose (2D) in /odom frame.
            fwd: True if most recent goal, when received, was the in 
                forward half-plane in the bot frame. Determines if the 
                bot will drive in forward or reverse direction toward 
                goal.
                
        Returns:
            Boolean which is True if goal has been reached.
        """
        if goal is None:
            return True
        d = self.getGoalDistance(cur, goal)
        dTh = abs(self.normalize_angle(cur.theta - goal.theta))
        return d < self.linearTolerance and dTh < self.angularTolerance
        
    def normalize_angle(self, a):
        """Normalizes an angle to range (-pi, pi].
        
        Args:
            a: An angle in radians.
            
        Returns:
            Normalized angle in radians in range (-pi, pi].
        """
        a = a % (2 * pi)
        if a > pi:
            a -= 2 * pi
        return(a)

    def getVelocity(self, cur, goal, fwd):
        """Computes linear and angular velocities to move toward a goal.
        
        Drives the robot forward if goal, when received, is in the 
        forward half-plane in the robot frame. Otherwise drives in 
        reverse. Uses feedback to try to reach a goal pose.
        
        See Introduction to Autonomous Mobile Robots by Siegwart et.al.
        
        Args:
            cur: Current pose (2D) from odometry in /odom frame.
            goal: Goal pose (2D) in /odom frame.
            fwd: True if most recent goal, when received, was the in 
                forward half-plane in the bot frame. Determines if the 
                bot will drive in forward or reverse direction toward 
                goal.
        
        Returns:
            desired: A "Pose" struct containing desired linear and 
            angular velocities in addition to position and orientation.
        """
        desired = Pose()
        d = self.getGoalDistance(cur, goal)
        
        tau = atan2(goal.y - cur.y, goal.x - cur.x)
        # treat forward and reverse driving differently
        if fwd:
            b = tau  # Siegwart sign convention
            a = tau - cur.theta
            linear_sign = 1
        else:
            b = pi + tau
            a = tau - cur.theta - pi
            linear_sign = -1
            
        a = self.normalize_angle(a)  # need to stay in (-pi, pi]
        b = self.normalize_angle(b)
        
        if abs(d) < self.linearTolerance:
            desired.xVel = 0
            # bot is at position so just spin to goal orientation
            # TODO need to fix magic numbers for speed limits
            desired.thetaVel = 1.0 * self.normalize_angle(goal.theta - cur.theta)
        else:
            desired.xVel = self.kP * d * linear_sign
            desired.thetaVel = self.kA*a + self.kB*b

        # Adjust velocities if linear or angular rates or accel too high.
        # TODO this is crude 
        if abs(desired.xVel) > self.maxLinearSpeed:
            desired.xVel = copysign(self.maxLinearSpeed, desired.xVel)
        if abs(desired.thetaVel) > self.maxAngularSpeed:
            desired.thetaVel = copysign(self.maxAngularSpeed, desired.thetaVel)
        # Adjust if speeds are too low abs value
        if abs(desired.xVel) < self.deadbandLinearSpeed:
            desired.xVel = 0.0
        elif abs(desired.xVel) < self.minLinearSpeed:
            desired.xVel = copysign(self.minLinearSpeed, desired.xVel)
        if abs(desired.thetaVel) < self.deadbandAngularSpeed:
            desired.thetaVel = 0.0
        elif abs(desired.thetaVel) < self.minAngularSpeed:
            desired.thetaVel = copysign(self.minAngularSpeed, desired.thetaVel)
       
        # diagnostic TODO
        print('Errors (c-g): x: %.2f y: %.2f yaw: %.2f d: %.2f; a: %.2f b: %.2f fwd: %s, des.xV: %.2f des.tV: %.2f' % 
            (cur.x-goal.x, cur.y-goal.y, cur.theta-goal.theta, d, a, b, fwd, desired.xVel, desired.thetaVel))
        
        return desired
