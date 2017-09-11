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
        self.maxLinearAcceleration = 1E9
        self.linearTolerance = 0.025 # 2.5cm
        self.angularTolerance = 3/180*pi # 3 degrees

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
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return sqrt(diffX*diffX + diffY*diffY)

    def atGoal(self, cur, goal):
        if goal is None:
            return True
        d = self.getGoalDistance(cur, goal)
        dTh = abs(cur.theta - goal.theta)
        return d < self.linearTolerance and dTh < self.angularTolerance
        
    def normalize_angle(self, a):
        '''
        For param a in radians returns value in (-pi, pi]
        '''
        a = a % (2 * pi)
        if a > pi:
            a -= 2 * pi
        return(a)

    def getVelocity(self, cur, goal, dT):
        desired = Pose()
        d = self.getGoalDistance(cur, goal)
        a = atan2(goal.y - cur.y, goal.x - cur.x) - cur.theta
        a = self.normalize_angle(a)
        b = cur.theta + a - goal.theta
        b = self.normalize_angle(b)
        
        if abs(d) < self.linearTolerance:
            desired.xVel = 0
            desired.thetaVel = -self.kB * b
        else:
            desired.xVel = self.kP * d
            desired.thetaVel = self.kA*a + self.kB*b

        # Adjust velocities if linear or angular rates or accel too high.
        # TBD
        # crude version
        if abs(desired.xVel) > 0.2:
            desired.xVel = copysign(0.2, desired.xVel)
        if abs(desired.thetaVel) > 1.0:
            desired.thetaVel = copysign(1.0, desired.thetaVel)
        
        # diagnostic TODO
        #print('cur: %.2f\t%.2f\t%.2f; goal: %.2f\t%.2f\t%.2f; d: %.2f; xV: %.3f; tV: %.3f' \
        #    % (cur.x, cur.y, cur.theta, goal.x, goal.y, goal.theta, d, desired.xVel, desired.thetaVel))
        print('Errors (c-g): x: %.2f y: %.2f yaw: %.2f d: %.2f; a: %.2f b: %.2f des.xV: %.2f des.tV: %.2f' % 
            (cur.x-goal.x, cur.y-goal.y, cur.theta-goal.theta, d, a, b, desired.xVel, desired.thetaVel))
        
        return desired
