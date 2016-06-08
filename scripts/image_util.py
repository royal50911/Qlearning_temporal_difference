import cv2
import numpy as np
import os
from read_config import read_config
from PIL import Image,ImageFont,ImageDraw

goals = (read_config()["goal"])
walls = (read_config()["walls"])
pits = (read_config()["pits"])
goal_reward = (read_config()["reward_for_reaching_goal"])
pit_reward = (read_config()["reward_for_falling_in_pit"])
wall_reward = (read_config()["reward_for_hitting_wall"])

map_x = (read_config()["map_size"][0])
map_y = (read_config()["map_size"][1])
MAP_SHAPE = (((map_x * (150 + 4)) + 4), ((map_y * (150 + 4)) + 4), 3)

up = Image.open("../img/pit_q.jpg")
down = Image.open("../img/pit_q.jpg")
left = Image.open("../img/pit_q.jpg")
right = Image.open("../img/pit_q.jpg")
goal = Image.open("../img/goal_q.jpg")
wall = Image.open("../img/wall_q.jpg")
pit = Image.open("../img/pit_q.jpg")
grid_o = Image.open("../img/grid.jpg")

temp = []

img_map = {
    "WALL": wall,
    "PIT": pit,
    "GOAL": goal,
    "N": up,
    "S": down,
    "W": left,
    "E": right
}

height, width, layers = MAP_SHAPE

def save_image_for_iteration(policy_list, iteration):
    #Creating an empty map of white spaces
    temp = policy_list
    empty_map = np.zeros(MAP_SHAPE)
    empty_map.fill(255)
    for row in range(map_x):
        for col in range(map_y):
            new_pos_row = ((row + 1) * 4) + (row * 150)
            new_pos_col = ((col + 1) * 4) + (col * 150)
            check = False
            if(goals == [row,col]):
            	policy_list.pop(0)
            	policy_list.pop(0)
            	policy_list.pop(0)
            	policy_list.pop(0)
            	policy_list.pop(0)

            	goal.save("../img/temp.jpg")
            	goal_o = Image.open("../img/temp.jpg")
            	draw = ImageDraw.Draw(goal_o)
            	font = ImageFont.truetype("../img/cool.ttf", 20)
            	draw.text((65, 65),str(goal_reward),(0,0,255),font=font)
            	empty_map[new_pos_row : new_pos_row + 150, new_pos_col : new_pos_col + 150] = goal_o
            	continue
            for wall_ in walls:
            	if(wall_ == [row,col]):
            		wall.save("../img/temp.jpg")
            		wall_o = Image.open("../img/temp.jpg")
            		draw = ImageDraw.Draw(wall_o)
            		font = ImageFont.truetype("../img/cool.ttf", 20)
            		draw.text((65, 65),str(wall_reward),(0,0,255),font=font)
            		empty_map[new_pos_row : new_pos_row + 150, new_pos_col : new_pos_col + 150] = wall_o
            		check = True
            for pit_ in pits:
            	if(pit_ == [row,col]):
            		pit.save("../img/temp.jpg")
            		pit_o = Image.open("../img/temp.jpg")
            		draw = ImageDraw.Draw(pit_o)
            		font = ImageFont.truetype("../img/cool.ttf", 20)
            		draw.text((65, 65),str(pit_reward),(0,0,255),font=font)
            		empty_map[new_pos_row : new_pos_row + 150, new_pos_col : new_pos_col + 150] = pit_o
            		check = True
            if(check):
            	policy_list.pop(0)
            	policy_list.pop(0)
            	policy_list.pop(0)
            	policy_list.pop(0)
            	policy_list.pop(0)
            	
            	continue

            grid_o.save("../img/temp.jpg")
            grid = Image.open("../img/temp.jpg")
            draw = ImageDraw.Draw(grid)
            font = ImageFont.truetype("../img/cool.ttf", 17)
            
            
            #right
            draw.text((97, 65),str(round(policy_list.pop(0),3)),(255,255,255),font=font)
            #left
            draw.text((12, 65),str(round(policy_list.pop(0),3)),(255,255,255),font=font)
            
            #down
            draw.text((44, 110),str(round(policy_list.pop(0),3)),(255,255,255),font=font)
            #up
            draw.text((44, 10),str(round(policy_list.pop(0),3)),(255,255,255),font=font)

            if(policy_list.pop(0) == goal_reward):
            	draw.ellipse((60, 60, 88, 88), fill = 'blue', outline ='blue')

            empty_map[new_pos_row : new_pos_row + 150, new_pos_col : new_pos_col + 150] = grid
    cv2.imwrite("../saved_video/iteration_" + str(iteration) + ".jpg", empty_map)



def generate_video(no_of_iterations):
    video = cv2.VideoWriter("../saved_video/video.avi", cv2.cv.CV_FOURCC('m', 'p', '4', 'v'), 1, (width, height))
    #print no_of_iterations
    for i in range(159):
        file_name = "../saved_video/iteration_" + str(i) + ".jpg"
        img = cv2.imread(file_name)
        video.write(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR))
        #This removes the image after stitching it to the video. Please comment this if you want the images to be saved
        #os.remove(file_name)
    video.release()
