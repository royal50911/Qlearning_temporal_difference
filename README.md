# Qlearning_temporal_difference
Reinforcement learning using Q-learning algorithm in which the probabilistic models that we used to solve the MDP problem are not known and have to be learned. Compare the 'optimal' policies that we obtained using reinforcement learning with the true optimal policies when the model was perfectly known. With Q-learning, robot doesn't need to know if the next step will give the optimal value. The robot learns about the optimal policy even when it is not acting optimally. So, the big idea is that robot learns from every experience from every action it takes, and gradually it knows the optimal path to reach from start to goal. 

+ Brief Intro : https://www.youtube.com/watch?v=ud6lMj1NpR8
+ Final Report: https://docs.google.com/document/d/1JzL97HN8YwnoCyyVAaRaqu6Q05V_5FG0NsSsoDzPeKM

#
#Usage
To run our code:
+ First install Ros: http://wiki.ros.org/ROS/Installation
+ After finishing Ros installation, then clone or copy our code to catkin_ws/src
+ Open the terminal and change dir to catkin_ws and  run catkin_make (This step is just the initial setup, so you dont have to run everytime)
+ Then run: 
         
           roslaunch cse_190_assi_3 solution_python.launch
+ After hit the command, it will run Q-learning and also other 2 algorithms Astar and MDP (If you dont want Astar and MDP, just comment out the line Astar() and MDP() in robot.py file at the very end of the code). After finish running, it will output 3 json files: pathlist.json for Astar, policy_MDP_list.json for MDP and policy_QL_list.json. It also outputs pics of simulation of Q-learning.
+ There are 3 test cases ( 3 configurations) in the folder. The default case run without modifying the code, to run other test cases, you need to replace other configurations to orginal one (configuration file). You also need to change ros.sleep time in robot.py, because other configurations have the bigger map, so it takes more time to output all pictures for Qlearning simulation ( you should change ros.sleep(10) to ros.sleep(1000) in robot.py to be able to fully output all pictures of running)

# Results:
+ We run our Qlearning algorithm in 2 maps and it works as we expect it, and it gives us the optimal path which is similar to the result of MDP algorithm.

+ Map 1: Video demo: https://www.youtube.com/watch?v=KQPePp9noQU 

+ Map 2: Video demo: https://youtu.be/uRmjcVr7U68

# Team Members:

Hien Hoang
A11953048

Chuanqiao Huang
A91403096




