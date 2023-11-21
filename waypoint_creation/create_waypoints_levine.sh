#! /bin/bash
python create_waypoints.py -m levine.png -o output
python create_waypoints.py -m levine.png -a output/annot_interp.txt -o output
python transform_waypoints_to_map.py -m levine.png -w output/final_waypoints.txt -y levine.yaml