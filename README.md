# Qlearning_temporal_difference
Reinforcement learning using Q-learning algorithm in which the probabilistic models that we used to solve the MDP problem are not known and have to be learned. Compare the 'optimal' policies that we obtained using reinforcement learning with the true optimal policies when the model was perfectly known.

#
#Usage
To run our code:
+ First install Ros: http://wiki.ros.org/ROS/Installation
+ After finishing Ros installation, then clone or copy our code to catkin_ws/src
+ Open the terminal and run catkin_make (This step is just the initial step, so you dont have run everytime)
+ Then run: 
          roslaunch cse_190_assi_3 solution_python.launch
+ After hit the command, it will run Q-learning and also other 2 algorithms Astar and MDP (If you dont want Astar and MDP just comment out the line Astar() and MDP() in robot.py file at the very end of the code). After finish running, it will output pathlist.json for Astar, policy_MDP_list.json for MDP and policy_QL_list.json. It also outputs pics of simulation of Q-learning.
+ There are 3 test cases ( 3 configurations) in the folder. The default case run without modifying the code, to run other test cases, you need to replace other configurations to orginal one (configuration file). You also need to change ros.sleep time in robot.py, because other configurations have the bigger map, so it takes more time to output all pictures for Qlearning simulation ( you should change ros.sleep(10) to ros.sleep(1000) in robot.py to be able to fully output all pictures of running)
