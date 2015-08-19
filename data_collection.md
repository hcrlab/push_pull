# How to run PR2 push_pull Data Collection Demo

## Desktop computer

First open three terminals on your desktop.

### In Terminal 1:
Type:

```
ssh username@c1
```

You are now logged into the computer on the robot.
Then type:

```
robot users 
```

If no one else is using the robot you can then do:

```
robot claim
```

and then:

```
robot start
```

After waiting about 30 seconds:

```
roslaunch pr2_pick_main main_prereqs.launch
```

### In Terminal 2:
Type:

```
setrobot c1
roslaunch pr2_pick_main frontend.launch
```

### In Terminal 3:
Type:

```
ssh username@c1
```

If you are __exploring action parameters__:

```
rosrun pr2_pick_main main.py --explore
```

Otherwise, if you are actually __collecting data__:

```
rosrun pr2_pick_main main.py
```

If you would like to start at an arbitrary trial number (e.g. trial 14) to continue a previously started data collection:

```
rosrun pr2_pick_main main.py --trial_number 14
```

## Laptop computer for UI

Open two terminals.

### On Terminal 4:

```
setrobot c1
roslaunch rosbridge_server rosbridge_websocket.launch
```

### On Terminal 5:

```
setrobot c1
roscd pr2_pick_main/web/web_interface
meteor
```

Open a browser and point to http://localhost:3000.


## Backing up collected data

Open a terminal and do the following.

```
ssh username@c1
roscd pr2_pick_main/data/experiments/
git add *.bag
git commit -am "adding next batch of collected data"
git push
```

## Updating software after changes

On every computer you are using (c1, desktop, laptop) do:

```
roscd push_pull
git pull
cd ../../../
catkin_make
```

## When you are done

Ctrl + C all terminal windows. On one of the terminals that was ssh'ed to c1 do:

```
robot stop
robot release
```




