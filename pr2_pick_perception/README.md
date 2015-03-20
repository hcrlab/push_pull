# Perception
Contains code for the perception side of the Amazon picking challenge.

## Actions
### GetItems
Given a bin ID (0-11), return a list of items, where each item includes a 6D pose.

## Libraries
### mock_perception.h
A mock perception library.
You explicitly specify where the items are in each bin.
Later, you can get the items back out.

## Executables
### mock_perception_easy_boxes.cpp
An action server for `GetItems` for the travel support demo.
It just tells the robot that the next object is always at more or less the same pose relative to the robot (since the robot drives to each bin).