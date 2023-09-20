#! /bin/bash
source .venv/bin/activate

# Build the base simulator image
docker build -t f1tenth_gym_ros -f f1tenth_gym_ros/Dockerfile .

# Use rocker to start the docker conatiner and add all the nodes as volumes
rocker --nvidia --x11 \
--volume f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros \
--volume f1tenth_abs/safety_node:/sim_ws/src/safety_node \
--volume f1tenth_wall_following/wall_follow:/sim_ws/src/wall_follow \
-- f1tenth_sim