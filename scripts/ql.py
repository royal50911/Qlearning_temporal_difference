#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import random as r
from read_config import read_config
from std_msgs.msg import Bool, String, Float32
from cse_190_assi_3.msg import PolicyList
import random


class QL():
	def __init__(self):
		self.init_ros_attribute()
		self.init_ros_functions()
		rospy.sleep(1)
		self.q_learning()
		#self.output_policy()
		
		
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
		self.threshold = self.config["threshold_difference"]
		self.iteration = self.config["max_iterations"]
		self.map_s = self.config["map_size"]		
		self.learn_rate = 0.8
		
		self.q_values = [[{(0, 1): 0.0, 
						   (0,-1): 0.0, 
						   (1, 0): 0.0, 
						   (-1,0): 0.0} for x in range(self.map_s[1])] for x in range(self.map_s[0])]
		
		
	def init_ros_functions(self):
		self.policy_pub = rospy.Publisher(
			"/results/policy_list",
			PolicyList,
			queue_size = 100
		)
		
	def grid_reward(self, x, y):
		reward = 0
		if self.isPit(x,y):
			reward += self.pit_reward
		if self.goal == [x,y]:
			reward += self.goal_reward
		if self.isWall(x, y):
			reward += self.wall_reward
		return reward	
		
	def isPit(self, x, y):
		for pit in self.pits:
			if pit == [x,y]:
				return True
		return False
			
	def isWall(self, x, y):
		for wall in self.walls:
			if wall == [x,y]:
				return True
		if self.isEdge(x,y):
			return True		
		return False
		
	def isEdge(self, x, y):
		row = self.map_s[0]
		col = self.map_s[1]
		if x >= row or x < 0 or y >= col or y < 0:
			return True
		return False
			
	def absorb_state(self, x, y):
		if [x,y] == self.goal:
			return True
		for pit in self.pits:
			if pit == [x,y]:
				return True
		return False
		
		
	def q_learning(self):
		episode = 0
		max_diff = 1
		iteration = 0
		#loop throught every episode
		while episode < self.max_iter and max_diff > self.threshold:
			max_diff = 0.0
			[x, y] = self.start
		
			#when x, y is not absorbing state, in one episode
			while not self.absorb_state(x, y):
				#select one among all possible actions for the current state.
				
				temp = self.q_values[x][y].items()
				random.shuffle(temp)
				i, j= max(temp, key=lambda x: x[1])[0]
				#print temp

				#for value in self.q_values[x][y].values():
				next_x, next_y = x+i, y+j
				old_q_value = self.q_values[x][y][(i,j)]
			
				#if the next state is wall, stay in the origin state
				if self.isWall(x+i, y+j):
					next_x, next_y = x, y
		
				#compute Q(s, a)
				utility = self.step_reward+self.grid_reward(x+i,y+j)+self.discount*max(self.q_values[next_x][next_y].values())
				self.q_values[x][y][(i,j)] = (1.0 - self.learn_rate)*old_q_value + self.learn_rate*utility
				
				#diff is a flog to terminate the loop
				diff = abs(self.q_values[x][y][(i,j)] - old_q_value)
				if max_diff < diff:
					max_diff = diff
				self.output_policy([x,y])
				iteration +=1
				#print "(x, y) = ", x, y,"motion = ",i,j," old q = ",old_q_value, " utility = ", utility ," new q = ", self.q_values[x][y][(i,j)]
				x, y = next_x, next_y
			episode += 1




		print "# of episode ", episode
		print "# of iteration", iteration
			
	def output_policy(self,pos):
		sign = None
		output = []
		"""for x in range(self.map_s[0]):
			#print
			
			for y in range(self.map_s[1]):
				if self.isWall(x, y):
					sign = "WALL"
				elif self.isPit(x, y):
					sign = "PIT"
				elif [x, y] == self.goal:
					sign = "GOAL"
				else:
					i, j = max(self.q_values[x][y], key=self.q_values[x][y].get)
					if [i, j] == [0,  1]:
						sign = "E"
					if [i, j] == [0, -1]:
						sign = "W"	
					if [i, j] == [1,  0]:
						sign = "S"
					if [i, j] == [-1, 0]:
						sign = "N"
						
				output.append(sign)
				#print sign,"""
		for x in range(self.map_s[0]):			
			for y in range(self.map_s[1]):
				values = self.q_values[x][y].values()
				#print x , y , self.q_values[x][y].values()
				for value in values:
					output.append(value)
					#print value	,
		#print "testing output", output
				if([x,y]==pos):
					output.append(self.goal_reward)
				else:
					output.append(0)

		self.policy_pub.publish(output)
		
