desktop:

roslaunch pr2_pick_manipulation move_group.launch


c1:

roslaunch pr2_pick_main main_prereqs.launch

rosrun pr2_pick_main main.py --explore

desktop:

roslaunch pr2_pick_main frontend.launch

roslaunch rosbridge_server rosbridge_websocket.launch

roscd push_pull/pr2_pick_main/web/web_interface;meteor
