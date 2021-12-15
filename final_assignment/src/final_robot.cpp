/**
 * Robotics control in a simulated environment
 * Robotics Engineering
 * 
 * @file final_robot.cpp
 * @author Simone Contorno (@simone-contorno)
 * 
 * @copyright Copyright (c) 2021
 * 
 * To exec the program run (in different shells):
 * roslaunch final_assignment simulation_gmapping.launch 
 * roslaunch final_assignment move_base.launch 
 * rosrun final_assignment final_robot
 */

// Headers
#include <csignal>
#include <iostream>
#include <string>
#include <chrono>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

// Declare the publishers
ros::Publisher pub_goal;
ros::Publisher pub_canc;
ros::Publisher pub_vel;

// Declare the subscribers
ros::ServiceClient go_to_point;
ros::ServiceClient keyboard;
ros::ServiceClient both;

// Global variables
float lin_vel = 0.0; // Robot linear velocity 
float ang_vel = 0.0; // Robot angular velocity
char key; // User input from keyboard to drive the robot

int drive_flag = 0; // Enable / Disable driving assistance
int time_flag = 0; // Compute the time elapsed since the request of the current goal

int flag = 0; // Just to manage printing of the option Enable / Disable driving assistance
int print_flag = 0; // Just to manage printing in drivingAssistance
int counter1 = 10; // Just to manage printing in manualDriving
int counter2; // Just to manage printing in userInterface

std::string id; // Goal ID
std::chrono::high_resolution_clock::time_point t_start;  
std::chrono::high_resolution_clock::time_point t_end; 

#define DIST 0.35 // Minimum distance from the wall with the driving assistance enabled

/**
 * Function: provide an user interface to drive the robot independently.
 * A driving assistance can be enabled (or clearly disabled).
 * 
 */
void manualDriving() {
    // Local variables
    geometry_msgs::Twist robot_vel;
    counter2 = 10;
    key = 'e';

    // Starting message
    printf("\n--- Manual Control ---\n\n"
    "Linear velocity advised: 0.5\n"
    "Angular velocity advised: 1.0\n");

    while (key != 'q') {
        // Commands list
        if (counter2 % 10 == 0) {
            printf("\nCommands:\n"
            "w - Go on\n"
            "a - Turn left\n"
            "s - Go back\n"
            "d - Turn right\n"
            "--------------\n"
            "z - Increase linear velocity\n"
            "x - Decrease linear velocity\n"
            "c - Increase angular velocity\n"
            "v - Decrease angular velocity\n"
            "-----------------------------\n"
            "e - Emergency stop\n"
            "q - Quit\n");
            
        }
        if (flag == 0) 
            printf("h - Enable driving assistance\n");
        else if (flag == 1)
            printf("h - Disable driving assistance\n");

        // Take user input
        printf("\nCommand: ");
        std::cin >> key;

        print_flag = 0;

        if (key == 'w') { // Go on
            robot_vel.linear.x = lin_vel;
            robot_vel.angular.z = 0; 
        }
        else if (key == 'a') { // Turn left
            robot_vel.linear.x = lin_vel;
            robot_vel.angular.z = +ang_vel; 
        }
        else if (key == 's') { // Go back
            robot_vel.linear.x = -lin_vel;
            robot_vel.angular.z = 0; 
        }
        else if (key == 'd') { // Turn right
            robot_vel.linear.x = lin_vel;
            robot_vel.angular.z = -ang_vel; 
        }
        else if (key == 'z') { // Increase linear velocity
            lin_vel += 0.1;
        }
        else if (key == 'x') { // Decrease linear velocity
            lin_vel -= 0.1;
        }
        else if (key == 'c') { // Increase angular velocity
            ang_vel += 0.1;
        }
        else if (key == 'v') { // Decrease angular velocity
            ang_vel -= 0.1;
        }
        else if (key == 'e') { // Emergency stop
            robot_vel.linear.x = 0;
            robot_vel.angular.z = 0; 
        }
        else if (key == 'h') { // Enable / Disable driving assistance
            if (flag == 0) {
                drive_flag = 1;
                flag = 1;
            }
            else if (flag == 1) {
                drive_flag = 0;
                flag = 0;
            }
        }
        else if (key == 'q') { // Quit
            robot_vel.linear.x = 0;
            robot_vel.angular.z = 0; 
            pub_vel.publish(robot_vel);
            counter1 = 10;
            break;
        }
        
        // Update message
        printf("Linear velocity: %f\n"
        "Angular velocity: %f\n", lin_vel, ang_vel);
        pub_vel.publish(robot_vel);
        counter2++;
    }
}

/**
 * ROS Callback Function: check data from robot's laser scanner and,
 * if the driving assistance is enable, help the user to not crush 
 * the robot against a wall. 
 * 
 * @param msg 
 */
void drivingAssistance(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Local variables
    float left = 30.0;
    float mid = 30.0;
    float right = 30.0;
    int i;
    int block = 0;
    geometry_msgs::Twist robot_vel;
    
    // Take the minimum values
    for (i = 0; i < 360; i++) { // On the right
        if (msg->ranges[i] < right)
            right = msg->ranges[i];
    }
    for (i = 300; i < 420; i++) { // In the middle
        if (msg->ranges[i] < mid)
            mid = msg->ranges[i];
    }
    for (i = 360; i < 720; i++) { // On the left
        if (msg->ranges[i] < left)
            left = msg->ranges[i];
    }

    // Driving assistance
    if (drive_flag == 1 & ((mid < DIST & key == 'w') || (left < DIST & key == 'a') || (right < DIST & key == 'd'))) {
        if (print_flag == 0) {
            printf("The robot is too close to the wall!\n");
            print_flag = 1;
        }
        robot_vel.linear.x = 0;
        robot_vel.angular.z = 0;
        pub_vel.publish(robot_vel);
    }

    // Check for the max time available to reach a goal point
    if (time_flag == 1) {
        t_end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        if (time > 120000000) {
            actionlib_msgs::GoalID canc_goal;
            printf("\nMax time elapsed! The goal point can't be reached!\n");
            canc_goal.id = id;
            pub_canc.publish(canc_goal);
            printf("Goal cancelled.\n");
            time_flag = 0;
        }
    }   
}

/**
 * ROS Callback Function: check the current Goal ID and save its in a global
 * variable.
 * 
 * @param msg 
 */
void currentGoalID(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    id = msg->status.goal_id.id;
}

/**
 * ROS Callback Function: provide an user interface to choose the modality to 
 * drive the robot; these are:
 * 1. Automatic driving (insert the coordinates to reach)
 * 2. Manual driving (without driving assitance)
 * 3. Manual driving (with driving assistance)
 * 
 * The driving assistance can be simply be enabled or disabled.
 * 
 * @param msg 
 */
void userInterface(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Local variables
    move_base_msgs::MoveBaseActionGoal goal_pos;
    actionlib_msgs::GoalID canc_goal;
    int x, y;
    char in;

    // Choose the action
    do {
        in = '5'; // Just to avoid a bug
        // Print commands list
        if (counter1 % 10 == 0) {
            printf("\nChoose an action:\n"
            "0 - Exit\n"
            "1 - Insert new coordinates to reach\n"
            "2 - Cancel the current goal\n"
            "3 - Manual driving\n");
        }
        if (flag == 0) 
            printf("4 - Enable driving assistance\n");
        else if (flag == 1)
            printf("4 - Disable driving assistance\n");
        
        // Take user input
        printf("Action (type the corresponding number): ");
        std::cin >> in;

        // Check input
        if (in != '0' & in != '1' & in != '2' & in != '3' & in != '4') 
            printf("\nERROR: type '0','1', '2', '3' or '4'.\n");

        counter1++;
    } while (in != '0' & in != '1' & in != '2' & in != '3' & in != '4');

    // Terminate the program
    if (in == '0') {
        printf("\n");
        exit(0);
    }

    // Insert new coordinates to reach
    else if (in == '1') {
        // Take coordinates to reach by the user
        printf("\nInsert coordinates to reach:\n");
        printf("X: ");
        std::cin >> x;
        printf("Y: ");
        std::cin >> y;

        t_start = std::chrono::high_resolution_clock::now();
        time_flag = 1;

        // Set new coordinates to reach
        goal_pos.goal.target_pose.header.frame_id = "map";
        goal_pos.goal.target_pose.pose.orientation.w = 1;
        
        goal_pos.goal.target_pose.pose.position.x = x;
        goal_pos.goal.target_pose.pose.position.y = y;

        // Publish new goal 
        pub_goal.publish(goal_pos);
    }
    
    // Cancel the current goal
    else if (in == '2') {
        // Take the Goal ID from the global variables continuosly
        // updated by the ROS Callback Function currentGoalID
        canc_goal.id = id;
        pub_canc.publish(canc_goal);
        printf("Goal cancelled.\n");
    }

    // Manual drive
    else if (in == '3') {
        canc_goal.id = id;
        pub_canc.publish(canc_goal);
        manualDriving();
    }

    // Enable or Disable drive assistance
    else if (in == '4') {
        if (flag == 0) {
            drive_flag = 1;
            flag = 1;
            printf("\nDriving assistance enabled.\n");
        }
        else if (flag == 1) {
            drive_flag = 0;
            flag = 0;
            printf("\nDriving assistance disabled.\n");
        }
    }
}

int main(int argc, char **argv) {
    printf("\nIMPORTANT:\n"
    "If you insert a string of characters, these will be analysed one by one.\n"
    "This can be useful if you want to increment the robot velocity with an unique\n"
    "command (e.g. zzzzz will be increment the velocity by 0.5), but it can be\n"
    "annoying if the command are not valid or if they are not Increment/Decrement commands.\n\n"
    "A goal will be cancelled if its will be not reached within 120 seconds.\n");

    // Initialize the node
    ros::init(argc, argv, "final_robot");
    ros::NodeHandle nh;
    
    // Define the publishers
    pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
    pub_canc = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    // Define the subscribers
    ros::Subscriber sub = nh.subscribe("/scan", 1000, userInterface);
    ros::Subscriber sub_id = nh.subscribe("/move_base/feedback", 1000, currentGoalID); // Current Goal feedback
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, drivingAssistance); // Laser scanner

    // Multi-threading
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}