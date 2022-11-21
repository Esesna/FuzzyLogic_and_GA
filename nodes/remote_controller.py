#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_control')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, GetModelState
from rosgraph_msgs.msg import Clock

from deap import base
from deap import creator
from deap import tools
from deap import base, algorithms

import random

import numpy as np
import math
from fuzzy_logic import FuzzyLogic
import os
import time

class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y
 
    def __sub__(self, P):
        return Point(self.x - P.x, self.y - P.y)

class Robot:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist)
        self.fuzzy_logic = FuzzyLogic()
        self.target = Point(0, 0)
        self.vel_msg = Twist()
        self.timeS = 0
        self.timeMS = 0
        self.timeStartMS = 0
        self.READY = True
        self.START = False
        self.STOP = False
        self.reward = 0

    def setTarget(self, P):
        self.target = P

    def callbackLaser(self, data):     # LaserScan
        self.updateModelState()
        self.laser_data = np.array(data.ranges)
        self.control()
    
    def callbackTime(self, data):     # LaserScan
        self.updateModelState()
        self.timeS = data.clock.secs
        self.timeMS = data.clock.nsecs / (1000 * 1000) + data.clock.secs * 1000

    def listener(self):
        rospy.Subscriber("/two_wheel_robot/laser/scan", LaserScan, self.callbackLaser, queue_size = 1)
        rospy.Subscriber("/clock", Clock, self.callbackTime, queue_size = 1)
        
    def updateModelState(self):

        rospy.wait_for_service('/gazebo/get_model_state')
        self.model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        self.object_coordinates = self.model_coordinates("two_wheel_robot", "ground_plane")

        z_pos = self.object_coordinates.pose.position.z
        y_pos = self.object_coordinates.pose.position.y
        x_pos = self.object_coordinates.pose.position.x
        self.Pos2d = Point(x_pos, y_pos)

        x_orient = self.object_coordinates.pose.orientation.z
        y_orient = self.object_coordinates.pose.orientation.y
        z_orient = self.object_coordinates.pose.orientation.x
        w_orient = self.object_coordinates.pose.orientation.w

        r = math.atan2(2*(w_orient * x_orient + y_orient * z_orient),1 - 2 * (x_orient * x_orient + y_orient * y_orient))
        p = math.asin(2*(w_orient * y_orient - z_orient * z_orient))
        y = math.atan2(2*(w_orient * z_orient + x_orient * y_orient),1 - 2 * (z_orient * z_orient + y_orient * y_orient))

        self.angleR = r*180/math.pi
        self.angleP = p*180/math.pi
        self.angleY = y*180/math.pi

    def angleVector(self, P1):
        moduleP = math.sqrt(P1.x * P1.x + P1.y * P1.y)
        cos = P1.x / moduleP
        sin = P1.y / moduleP
        angle = math.acos(cos)
        if sin < 0:
            angle *= -1
        return angle / math.pi * 180

    def talker(self, event):
        self.pub.publish(self.vel_msg)

    def setNullSpeed(self):
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.linear.w = 0

        self.pub.publish(self.vel_msg)


    def startDefaultModel(self, Ps):
        state_msg = ModelState()
        state_msg.model_name = 'two_wheel_robot'
        state_msg.pose.position.x = Ps.x
        state_msg.pose.position.y = Ps.y
        state_msg.pose.position.z = 0

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        self.READY = True
        self.START = False
        self.STOP = False
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def endCondition(self, distance, time):
        if(time > 72000):
            self.reward = 0
            return True

        if(distance < 0.05):
            self.reward = 72000 - time
            return True

        return False

    def control(self):

        start = time.time()
        min_right = self.laser_data[0:8].min()
        min_front = self.laser_data[8:28].min()
        min_left = self.laser_data[28:36].min()

        newP = self.target - self.Pos2d
        angle = self.angleVector(newP) - self.angleR
        angle = -angle
        if(angle < -180): 
            angle = 360 + angle
        if(angle > 180): 
            angle = -360 + angle

        distance = math.sqrt(newP.x * newP.x + newP.y * newP.y)
        if(min_front == np.inf and min_right == np.inf and min_left == np.inf):
            vel, rot = self.fuzzy_logic.computeTFLC(angle, distance)
        else:
            if(min_left > 2):
                min_left = 2
            if(min_front > 2):
                min_front = 2
            if(min_right > 2):
                min_right = 2
            vel, rot = self.fuzzy_logic.computeOAFLC(min_left, min_front, min_right)

        finish = time.time()

        os.system('cls||clear')
        print("Input: ", min_right, min_front, min_left)
        print("Path to: ", angle, distance)
        print("Output: ", vel, rot)
        print("Time: ", self.timeMS - self.timeStartMS, " ms")
        print("Time for ",finish - start)

        if(self.endCondition(distance, self.timeMS - self.timeStartMS)):
            if(not self.STOP):
                self.STOP = True
                self.finishTime = self.timeMS - self.timeStartMS

            print("FinishTime: ", self.finishTime, " ms")
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        else:
            self.vel_msg.linear.x = vel
            self.vel_msg.angular.z = rot

        if(not self.START and self.READY):
            self.START = True
            self.READY = False
            self.timeStartMS = self.timeMS


def main():

    # INIT ROBOT
    rospy.init_node("Control_robot")
    robot = Robot()
    robot.setTarget(Point(1, 0))
    robot.startDefaultModel(Point(-8, 0))

    timer = rospy.Timer(rospy.Duration(0.5), robot.talker)

    robot.listener()
    # time.sleep(100000)
    def Run(listBit):
        robot.fuzzy_logic.reInit(listBit)
        robot.startDefaultModel(Point(-8, 0))
        while(not robot.STOP):
            time.sleep(0.1)
        return (robot.reward, )

    # INIT GA
    ONE_MAX_LENGTH = robot.fuzzy_logic.n * robot.fuzzy_logic.countParam # Size list bit
    POPULATION_SIZE = 20 # Size population
    P_CROSSOVER = 0.9
    P_MUTATION = 0.1
    MAX_GENERATIONS = 20
    # creator.create("Point", object, MAX_COORD = 10, MIN_COORD = 0, coords=list)

    # pt = creator.Point()
    creator.create("FitnessMax", base.Fitness, weights = (1.0, ))
    creator.create("Individual", list, fitness = creator.FitnessMax)


    toolbox = base.Toolbox()
    toolbox.register("randRange", random.randint, 0, 1)
    toolbox.register("individualCreator", tools.initRepeat, creator.Individual, toolbox.randRange, ONE_MAX_LENGTH)
    toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)

    population = toolbox.populationCreator(n=POPULATION_SIZE)

    np.savetxt("first_population.csv", population, delimiter=",")

    toolbox.register("select", tools.selTournament, tournsize=3)
    toolbox.register("mate", tools.cxOnePoint)
    toolbox.register("mutate", tools.mutFlipBit, indpb=1.0/ONE_MAX_LENGTH)
    toolbox.register("evaluate", Run)

    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("max", np.max)
    stats.register("avg", np.mean)

    population, logbook = algorithms.eaSimple(population, toolbox,
                                            cxpb=P_CROSSOVER,
                                            mutpb=P_MUTATION,
                                            ngen=MAX_GENERATIONS,
                                            stats=stats,
                                            verbose=True)

    maxFitnessValues, meanFitnessValues = logbook.select("max", "avg")
    
    np.savetxt("maxFitnessValues.csv", maxFitnessValues, delimiter=",")
    np.savetxt("meanFitnessValues.csv", meanFitnessValues, delimiter=",")
    np.savetxt("population.csv", population, delimiter=",")

    rospy.spin()
    timer.shutdown()

if __name__ == "__main__":
    main()


