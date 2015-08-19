# How to run PR2 push_pull Data Collection Demo

## Desktop computer

First open three terminals on your desktop.

### In Terminal 1:
Type:

> ssh username@c1

You are now logged into the computer on the robot.
Then type:

> robot users 

If no one else is using the robot you can then do:

> robot claim

and then:

> robot start

After waiting about 30 seconds:

> roslaunch pr2_pick_main main_prereqs.launch

### In Terminal 2:
Type:

> setrobot c1

> roslaunch pr2_pick_main frontend.launch

### In Terminal 3:
Type:

> ssh username@c1

If you are __exploring action parameters__:

> rosrun pr2_pick_main main.py --explore

Otherwise, if you are actually collecting data:

> rosrun pr2_pick_main main.py

## Laptop computer for UI

Open two terminals.

### On Terminal 4:

> setrobot c1

> roslaunch rosbridge_server rosbridge_websocket.launch

### On Terminal 5:

> setrobot c1

> roscd push_pull/pr2_pick_main/web/web_interface

> meteor

Open a browser and point to http://localhost:3000.

