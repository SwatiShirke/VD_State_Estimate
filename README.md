# VD state estimation

This repo consist of code for real time motion planning for Autonomous Vehicle with simulation in Carla Engine. 

#start carla with ros arg from the dir where carla is installed
cd carla_sim10/Carla-0.10.0-Linux-Shipping/
source ./CarlaUnreal.sh --ROS2

#go to the MP_for_AV dir, start carla client node with pygame, vehicle is spawned here
cd carla_client 
cd Adv_Robot_Nav/VD_State_Estimation/src/VD_State_Estimate/carla_client/
python3 launch_carla_client.py



#start nodes 
cd Adv_Robot_Nav/VD_State_Estimation/
source install/setup.bash


#launch
ros2 launch vd_state_estimate state_estimator_launch.py




