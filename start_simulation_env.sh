#! /bin/bash
source .venv/bin/activate

# Build the base simulator image
docker build -t f1tenth_gym_ros -f f1tenth_gym_ros/Dockerfile .

# Use rocker to start the docker conatiner and add all the nodes as volumes
rocker --nvidia --x11 \
--volume f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros \
--volume f1tenth_abs/safety_node:/sim_ws/src/safety_node \
--volume f1tenth_wall_following/wall_follow:/sim_ws/src/wall_follow \
--volume f1tenth_follow_the_gap/gap_follow:/sim_ws/src/gap_follow \
--volume f1tenth_pure_pursuit/pure_pursuit:/sim_ws/src/pure_pursuit \
--volume f1tenth_rrt/rrt:/sim_ws/src/rrt \
--volume f1tenth_mpc/mpc:/sim_ws/src/mpc \
--volume f1tenth_waypoint_publisher/waypoint_publisher:/sim_ws/src/waypoint_publisher \
-- f1tenth_gym_ros