#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import random as r
from Queue import *
from read_config import read_config
from std_msgs.msg import Bool, String, Float32
from cse_190_assi_3.msg import PolicyList,AStarPath


class Astar():
    def __init__(self):
        
        self.init_ros_attribute()
        self.init_ros_things()
        rospy.sleep(1)
        self.path_search()
    

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
        self.step_reward = self.config["reward_for_each_step"]
        self.discount= self.config["discount_factor"]
        self.prob_fwd= self.config["prob_move_forward"]
        self.prob_back= self.config["prob_move_backward"]
        self.prob_left= self.config["prob_move_left"]
        self.prob_right= self.config["prob_move_right"]
        self.threshold = self.config["threshold_difference"]
        self.iteration = self.config["max_iterations"]
        self.map_s = self.config["map_size"]

        self.fwd = [[0 for x in range(self.map_s[1])] for x in range(self.map_s[0])]
        self.back= [[0 for x in range(self.map_s[1])] for x in range(self.map_s[0])]
        self.astar = [[0 for x in range(self.map_s[1])] for x in range(self.map_s[0])]
        self.path = [[[1,1] for x in range(self.map_s[1])] for x in range(self.map_s[0])]
    
    
    # initial subscriber and publisher
    def init_ros_things(self):

        self.path_pub = rospy.Publisher(
            "/results/path_list",
            AStarPath,
            queue_size = 100,
        )



    def heuristic(self,data, start):

    	q = Queue()
    	q.put(start)
    	while not q.empty():
    		temp = q.get()
    		for move in self.movelists:
    			row = temp[0]+move[0]
    			col = temp[1]+move[1]
    			if(row >(self.map_s[0]-1) or col >(self.map_s[1]-1) or row <0 or col <  0):
    				continue
    			else:
    				if(data[row][col] == 0 and [row,col] != start):
    					data[row][col] = data[temp[0]][temp[1]] +1
    					q.put([row,col])

        
    def path_search(self):
    	self.heuristic(self.fwd,self.start)
    	self.heuristic(self.back,self.goal)

    	for i in range(self.map_s[0]):
    		for j in range (self.map_s[1]):
    			self.astar[i][j]= self.fwd[i][j] + self.back[i][j]
    			#print self.fwd[i][j],self.back[i][j]

    	for wall in self.walls:
    		self.astar[wall[0]][wall[1]] = -1
    	for pit in self.pits:
    		self.astar[pit[0]][pit[1]] = -1

    	"""for i in range(self.map_s[0]):
    		for j in range (self.map_s[1]):
    			print "astar ", self.astar[i][j]"""

    	pq = PriorityQueue()
    	pq.put(self.start,self.astar[self.start[0]][self.start[1]])
    	while not pq.empty():
    		temp = pq.get()
    		#print "test", self.astar[temp[0]][temp[1]]
    		if(self.astar[temp[0]][temp[1]] < 0):
    			continue
    		else:
    			self.astar[temp[0]][temp[1]] = -1
	    		for move in self.movelists:
	    			row = temp[0]+move[0]
	    			col = temp[1]+move[1]
	    			if(row >(self.map_s[0]-1) or col >(self.map_s[1]-1) or row <0 or col <  0):
	    				continue
	    			else:
	    				if(self.astar[row][col] <0):
	    					continue
	    				else:
	    					self.path[row][col] = temp
	    					pq.put([row,col],self.astar[row][col])

    	#for i in range(self.map_s[0]):
    		#for j in range (self.map_s[1]):

    			#print self.path[i][j]

    	reverse_path = []
    	q = Queue()
    	q.put(self.goal)
    	reverse_path.append(self.goal)
    	while not q.empty():
    		temp = q.get()
    		if(temp == self.start):
    			break
    		for move in self.movelists:
    			row = temp[0]+move[0]
    			col = temp[1]+move[1]
    			if(row >(self.map_s[0]-1) or col >(self.map_s[1]-1) or row <0 or col <  0):
    				continue
    			else:
    				if([row,col] == self.path[temp[0]][temp[1]]):
    					q.put([row,col])
    					reverse_path.append([row,col])

    	#reverse_path.append(self.start)
    	#real_path = []
    	#real_path.append(self.start)


    	for i in range (len(reverse_path)):
    		self.path_pub.publish(reverse_path[len(reverse_path)-1-i])
    		#print reverse_path[len(reverse_path)-1-i]

    	

if __name__ == '__main__':
    r = Astar()
