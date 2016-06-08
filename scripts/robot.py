#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import random as r
from read_config import read_config
from astar import Astar
from mdp import MDP
from ql import QL
from std_msgs.msg import Bool, String, Float32
from cse_190_assi_3.msg import AStarPath


class Robot():
    def __init__(self):
        rospy.init_node("Robot")
        self.init_ros_attribute()
        self.init_ros_things()
        self.search_exe()
        rospy.spin()
    

  #___________________________________________________#
  #
  #           Helper functions define below
  #___________________________________________________#
  
  
    def init_ros_attribute(self):
        self.config = read_config() 
        self.start = self.config["start"]
        self.goal = self.config["goal"] 
        self.walls = self.config["walls"]
        self.pits = self.config["pits"]
        self.max_iter = self.config["max_iterations"]
        self.movelists = self.config["move_list"]
        self.wall_reward = self.config["reward_for_hitting_wall"]
        self.goal_reward = self.config["reward_for_reaching_goal"]
        self.pit_reward = self.config["reward_for_falling_in_pit"]
        self.discount= self.config["discount_factor"]
        self.prob_fwd= self.config["prob_move_forward"]
        self.prob_back= self.config["prob_move_backward"]
        self.prob_left= self.config["prob_move_left"]
        self.prob_right= self.config["prob_move_right"]
        self.threshold = self.config["threshold_difference"]
        self.iteration = self.config["max_iterations"]


    # initial subscriber and pubilsher
    def init_ros_things(self):
        
        self.finish_pub = rospy.Publisher(
            "/map_node/sim_complete",
            Bool,
            queue_size = 100,
        )

    def search_exe(self):
        
        #Astar()
        #self.path_pub.publish(path)
        #MDP()
        QL()
        self.finish_pub.publish(True)
        rospy.sleep(10)
        rospy.signal_shutdown("Finish Simulation")     
  
        
if __name__ == '__main__':
    r = Robot()