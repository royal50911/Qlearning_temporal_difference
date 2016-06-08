#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import random as r
from read_config import read_config
from std_msgs.msg import Bool, String, Float32
from cse_190_assi_3.msg import PolicyList,PathList


class MDP():
    def __init__(self):
        
        self.init_ros_attribute()
        self.init_ros_things()
        rospy.sleep(1)
        self.find_optimal()
    

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

        self.map_reward = [[(0.0) for x in range(self.map_s[1])] for x in range(self.map_s[0])]
        self.map_policy = [[("w") for x in range(self.map_s[1])] for x in range(self.map_s[0])]
        #print "row ", len(self.map_reward)
        #print "col ", len(self.map_reward[0])

        self.diff = 0
        


    
    
    # initial subscriber and publisher

    def init_ros_things(self):
        
        self.policy_pub = rospy.Publisher(
            "/results/policy_mdp_list",
            PathList,
            queue_size = 100,
        )


    def find_max(self, cell):
    	maxvalue = -100.0
    	maxpolicy = 'W'
    	maxdiff =0
    	prob = 0
    	for wall in self.walls:
    		if(cell[0] == wall[0] and cell[1] == wall[1]):
    			return (self.wall_reward,"WALL")
    	for pit in self.pits:
    		if(cell[0] == pit[0] and cell[1] == pit[1]):
    			return (self.pit_reward,"PIT")
    	if(cell[0] == self.goal[0] and cell[1] == self.goal[1]):
    		return (self.goal_reward,"GOAL")


    	for move1 in self.movelists:
    		total =0
    		policy = "W"
    		row = col =0

    		for move2 in self.movelists:
    			temp = 0
    			if(move1 == move2):
    				prob = self.prob_fwd
    			elif (move1[0]==-move2[0] or move1[1]==-move2[1] ):
    				prob = self.prob_back
    			else:
    				prob = self.prob_left

	    		row = cell[0] + move2[0]
	    		col = cell[1] + move2[1]
	    		pit_check = False
	    		wall_check = False
	    		goal_check = False

	    		if(row >(self.map_s[0]-1) or col >(self.map_s[1]-1) or row <0 or col <  0):
	    			wall_check = True
	    			#print "over bound"
	    		else:
		    		for wall in self.walls:
		    			if(row == wall[0] and col == wall[1]):
		    				wall_check = True
		    				#print "wall "
	    		for pit in self.pits:
	    			if(row == pit[0] and col == pit[1]):
	    				pit_check = True
	    		if (row == self.goal[0] and col == self.goal[1]):
	    			goal_check = True

	    		#print "cell ", cell[0], cell[1]

	    		if(wall_check):
	    			temp = prob *(self.wall_reward + self.discount*self.map_reward[cell[0]][cell[1]])
	    			#print "wall reward" , temp, policy
	    		elif (pit_check):
	    			temp = prob *(self.step_reward + self.discount*self.pit_reward)
	    			#print "pit reward" , temp, policy
	    		elif (goal_check):
	    			temp = prob *(self.step_reward + self.discount*self.goal_reward)
	    			#print "goal reward" , temp, policy
	    		else:
	    			#print "cell ", row, col
	    			temp = prob *(self.step_reward + self.discount*self.map_reward[row][col])
			    	#print "else reward" , temp, policy

	    		if(move1 == move2):
		    		if(move1 == [0,1]):
		    			policy = "E"
		    		elif (move1 == [0,-1]):
		    			policy = "W"
		    		elif(move1 == [1,0]):
		    			policy = "S"
		    		elif(move1 == [-1,0]) :
		    			policy = "N"

	    		total += temp
	    		
	    	#print "total reward" , total, policy	
	    	if(maxvalue <= total):
	    		maxvalue = total
	    		maxpolicy = policy
	    		maxdiff= abs(self.map_reward[cell[0]][cell[1]] - maxvalue)

    	self.diff += maxdiff
    	#print "max value", maxvalue
    	#print "policy", maxpolicy

    	return (maxvalue, maxpolicy)

        
    def find_optimal(self):
        new_reward = [[(0.0) for x in range(self.map_s[1])] for x in range(self.map_s[0])]
        new_policy = [[("w") for x in range(self.map_s[1])] for x in range(self.map_s[0])]

        for k in range (10):
        	for i in range(self.map_s[0]):
        		for j in range(self.map_s[1]):
        			cell = [i,j]
        			maxvalue, maxpolicy = self.find_max(cell)
        			#print " done 1", maxvalue, maxpolicy, self.diff
        			new_reward[i][j] = maxvalue
        			new_policy[i][j] = maxpolicy

        	
        	temp = []
        	for i in range(self.map_s[0]):
        		for j in range(self.map_s[1]):
        			self.map_reward[i][j] = new_reward[i][j]
        			self.map_policy[i][j] = new_policy[i][j]
        			temp.append(self.map_policy[i][j])
        			#print self.map_policy[i][j]

        	self.policy_pub.publish(temp)

        	if(self.diff <= self.threshold):
        		self.diff = 0
        		break


        
if __name__ == '__main__':
    r = MDP()
      