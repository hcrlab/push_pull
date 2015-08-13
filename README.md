# push_pull
External things you need: 
- gripper_grasp_planner_cluster --Add links
- object_recognition thing
- convert_pcl 

If you find something missing, make sure launch file is right, might be some components in other launch files.

How to run it:
- run move_group on your machine
- run main_prereqs on the robot
- if it says "waiting for point cloud something... " run demo.launch from object_recognition
- rosrun main.py --plan_grasp (possible unnecessary later)

- will ask for bag file name
- bag files named after position and rotation

RVIZ: interactive markers, PC2, markers



