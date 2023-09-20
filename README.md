# F1 Tenth CMU Perception Stack

 This repo is a comprehensive list of ROS2 nodes that are developed for labs at part of the F1 Tenth Autonomous Racing class at Carnegie Mellon University.

 | Lab    | Location | Programmed In  |
| -------- | ------- | -------------- |
| Emergency Breaking | [f1tenth_abs](./f1tenth_abs/)    | Python |
| Wall Following | [f1tenth_wall_following](./f1tenth_wall_following/)   | C++ |
| Follow the Gap    | Not Finished   | Python |


## Entering into Simulation Environment

Install Rocker then build simulator with ROS2 nodes as volumes.
```sh
python -m venv .venv
pip install rocker
sudo ./start_simulation_env.sh
```