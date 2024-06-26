# Hornet X Software Assignment 2

This repository contains a ROS2 package called "assignment2". It includes a simulation node and a controller node.
Your task is to implement a 1-dimensional PID controller to control the simulated vehicle (orange box) in the simulation.

**DO NO MODIFY "simulation.py"** (but feel free to look inside)

Zip and email your solution to hornetxauv2425@gmail.com with subject [Software Homework 2] \<Name\>.

## Setting up the simulation
This simulation depends on pygame. To install pygame, either run
```bash
sudo apt install python3-pygame
```
or
```bash
pip3 install pygame
```
To run the simulation, add this package to your workspace and build
```bash
colcon build --packages-select assignment2
source install/setup.bash
ros2 run assignment2 simulation
```
## Troubleshooting no GUI output
If you are on WSL, you might need to set up the following to see a GUI output.

Download and install VcXsrv from https://sourceforge.net/projects/vcxsrv/. Run the application and
1. select multiple windows and display number -1
2. select start no client
3. select everything on the extra settings
4. finish

Which the xserver running, on your linux command line run
```bash
export DISPLAY=`grep -oP "(?<=nameserver ).+" /etc/resolv.conf`:0.0
```
Next test if you can see GUI output. We will use xeyes.
```bash
sudo apt install x11-apps
```
Then run 
```bash
xeyes
```
You should see a pair of googly eyes following your cursor.

## Using the simulation
The simulation publishes the following data in `std_msgs/Float64` format
```bash
/depth    # current vehicle depth
/setpoint # current setpoint
```
The simulation also subscribes to the following topic (also of `std_msgs/Float64` type)
```bash
/thrust   # -4 to 4, with upwards positive
```
Publishing to this topic will allow you to control the vertical thrust of the vehicle.

You can mouse click on the GUI to change the setpoint.

## Running the controller
Make the relevant changes to the controller node template. Then build, source and run 
```bash
ros2 run assignment2 controller
```
to test your controller.
