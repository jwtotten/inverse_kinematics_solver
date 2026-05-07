# Inverse Kinematics Solver

This repository is for solving the inverse kinematics of my hexapod robot.
This will demonstrate how to calculate the angles of the servos and illustrate this with an interactive plot.

# Current State of development

## Solutions to the inverse kinematic equations

Currently, I have been able to correctly calculate the inverse kinematic angles for a preselected array of positions that will move any of the 6 target legs forwards or backwards. From this, i can select if the list of positions that take the leg from A to b is traversed either forwards or backwards.

My goal going forward is to be able to verify that the legs move in either a forwards, backwards, left or right direction by introducing a gait_controller class that will preselect the array of target positions to move to based on inputs from the user.

This can be done either with a preset rotation to start with or keystroke detection to change the direction of movement of the robot.

## Mulit-leg animation

The multi-leg animation plot is currently able to animate all 6 legs concurrently. However, there is currently no input from the user to be able to specify whether or not the legs move in a forward or backwards direction.

My goal going forwards is to be able to dynamically change the direction the legs of the robot take from the inputs described above, and allow the user to change the speed of movement and simulation of the robots legs.

## Unittesting

Unittesting has been implemented to further debug and validate the inverse kinematic solver solutions, as well as be able to verify the gait controlling can be successfully altered.
