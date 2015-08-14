# Data Collection Instructions

First Open two terminals.

## In the first terminal: 
Type:
> ssh username@c1

You are now logged into the computer on the robot.
Then type:
> robot users 

If no one else is using the robot you can then do:
> robot claim

and then:

> robot start

## In the second terminal:
Type:
> roslaunch pr2_pick_manipulation move_group.launch

## In the first terminal again:
Type:
> roslaunch pr2_pick_main main.launch
