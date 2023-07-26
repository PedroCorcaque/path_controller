#!/usr/bin/env python3

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

class Env():
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.goal_numbers = 10
        self.collision_numbers = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13 # bateu
        collision = False
        goal = False
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
        
        if min_range > min(scan_range) > 0:
            collision = True
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            goal = True
            done = True

        return scan_range, current_distance, collision, goal, done
    
    def shutdown(self):
        rospy.loginfo("Terminado atividade do TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.signal_shutdown("The End")
    
    def report_goal_distance(self, distance, collision, goal, step=1.0):
        reward = -np.log(self.getGoalDistace() + 1e-8) * step * 0.01
        if collision:
            rospy.loginfo("Collision!!")
            self.collision_numbers += 1
            reward = -100
        if goal:
            rospy.loginfo("Goal!!")
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            distance = self.goal_distance
            rospy.loginfo("Target position: x-> %s, y-> %s!!", self.goal_x, self.goal_y)
            self.goal_numbers -= 1
            reward = 200
        
        rospy.loginfo("Number of targets %s / distance to curent goal %s / collission number %s", self.goal_numbers, distance, self.collision_numbers)
        return reward
        
    def step(self, action, step):
        liner_vel = action[0]
        ang_vel = action[1]
        
        if self.goal_numbers == 0:
            self.shutdown()

        vel_cmd = Twist()
        if liner_vel < 0.1:
            liner_vel = 0.1
        elif liner_vel > 0.5:
            liner_vel = 0.5
        if ang_vel < -0.5:
            ang_vel = -0.5
        elif ang_vel > 0.5:
            ang_vel = 0.5
        vel_cmd.linear.x = liner_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, distance, collision, goal, done = self.getState(data)
        
        reward = self.report_goal_distance(distance, collision, goal, step)

        return np.append(np.asarray(state), self.getGoalDistace()), reward, done, {}

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(position_check=True)
            
        rospy.loginfo("Target position: x-> %s, y-> %s!!", self.goal_x, self.goal_y)

        self.goal_distance = self.getGoalDistace()
        state, distance, collision, goal, done = self.getState(data)
        
        reward = self.report_goal_distance(distance, collision, goal)

        return np.append(np.asarray(state), self.getGoalDistace()), {}
