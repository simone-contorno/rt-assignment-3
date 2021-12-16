# RT-Assignment-3
## Third Assignment of Research Track 1 - Robotics Engineering
### Author: Simone Contorno

<br>

Control of a robot in a simulated environment.

### Introduction
An overview of this program function.<br>
[Go to Introduction](#intro)

### How it works
A rapid description of how the program works (pseudo-code).<br>
[Go to How it works](#how)

### Installation and Execution
How install and run RT-Assignment-3 in Linux.<br>
[Go to Installation and Execution](#installation)

### Improvements
How this program could be improved.<br>
[Go to Improvements](#improve)

<a name="intro"></a>
### Introduction

This program manage a robot, endowed with laser scanners, which should move autonomously inside a map.<br>
You can use the user interface to:
<ol>
    <li>Let the robot to autonomously reach a x,y coordinate inserted by command line.</li>
    <li>Drive the robot with the keyboard.</li>
    <li>Drive the robot with the keyboard availing of a simple driving assistance.</li>
</ol>

The map is this one:<br>
<br>Rviz:<br>
<img src="https://github.com/simone-contorno/RT-Assignment-3/blob/main/third_assignment_map_rviz.png" width="275" height="377">
<br><br>Gazebo:<br>
<img src="https://github.com/simone-contorno/RT-Assignment-3/blob/main/third_assignment_map_gazebo.png" width="500" height="259">

<a name="how"></a>
### How it works

The program use the launch file "simulation_gmapping.launch", to run the simulated environment, and the launch file "move_base.launch" to run the action move_base that provides several topics, including:
<ul>
    <li>move_base/goal to publish the goal position;</li>
    <li>move_base/feedback to receive the feedback;</li> 
    <li>move_base/cancel to cancel the current goal.</li>
</ul>
<br>
There are 2 subscribers that run simultaneously thanks to a multi-thread architecture given by the ROS class AsyncSpinner:
<ul>
    <li>sub_id: subscribe to the topic /move_base/feedback through the function currentGoalID that continuosly update the current goal ID.</li>
    <li>sub_laser: subscribe to the topic /scan through the function drivingAssistance that continuosly take data by the lasar scanner and, if the driving assistance is enable, help the user to drive the robot, stopping its if there is a wall too close in the current direction.</li>
</ul>
<br>
The robot can:
<ol>
    <li>Autonomously reaching a goal position: 
        <ul>
            <li>ask to the user to insert the coordinates x and y to reach;</li>
            <li>save the current time;</li>
            <li>set the fram_id to "map" (corresponding to the environment that is used) and the new coordinates to reach;</li>
            <li>publish the new goal to move_base/goal.</li>
        </ul>
    </li>
    <li>Cancel the current goal:
        <ul>
            <li>take the current goal ID;</li>
            <li>publish its to the topic move_base/cancel.</li>
        </ul>
    </li>
    <li>Be driven by the user through the keyboard (the list of commands is printed on the console).</li>
</ol>

Look the pseudocode files for more details.<br>

<a name="installation"></a>
### Installation and Execution

Open the terminal, and download this repository:

<pre><code>git clone https://github.com/simone-contorno/RT-Assignment-3</code></pre>

Copy or move the folder final_assignment into the src folder of your ROS workspace.<br> 
Go into the root folder of your ROS workspace and type: 

<pre><code>catkin_make</code></pre>

Afterwards type:

<pre><code>rospack profile</code></pre>

Now, open 3 terminals; in the first one launch the environment:

<pre><code>roslaunch final_assignment simulation_gmapping.launch</code></pre>

In the second one launch the action move_base:

<pre><code>roslaunch final_assignment move_base.launch</code></pre>

In the third one run the node final_robot:

<pre><code>rosrun final_assignment final_robot</code></pre>

<a name="improve"></a>
### Improvements

The driving assistance can be improved by move the robot in the right direction when the user is driving 
its against a wall, instead of just stop it.<br><br>

Thanks to have read this file, i hope it was clear and interesting.<br>