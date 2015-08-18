# Terminal Windows

## Desktop:

roslaunch pr2_pick_manipulation move_group.launch

## c1:

roslaunch pr2_pick_main main_prereqs.launch

rosrun pr2_pick_main main.py --explore

## Desktop:

(optional) roslaunch pr2_pick_main frontend.launch

### (If you want to run the website from the laptop do these on the laptop:)

roslaunch rosbridge_server rosbridge_websocket.launch

roscd push_pull/pr2_pick_main/web/web_interface;meteor

# Chrome

Open a window and go to localhost:3000
