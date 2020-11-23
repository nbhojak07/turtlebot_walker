/**
 *  Copyright 2020 Nidhi Bhojak
 *  @file walker.cpp
 *  @author Nidhi Bhojak
 *  @date 11/18/2020
 *
 *  @brief Walker class 
 *
 *  @section LICENSE
 *  
 * MIT License
 * Copyright (c) 2020 Nidhi Bhojak
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *  @section DESCRIPTION
 *
 *  Header file for Walker Algorithm Class 
 *
 */
#include <ros/ros.h>
#include "walker.hpp"

/** 
 * @brief Constructor for the Walker Class
 * **/
Walker::Walker() {
    // Initialise the Collision check variable 
    checkCollision = false;
}

/** 
 * @brief Destructor for the Walker Class
 * **/

Walker::~Walker() {
    // Value initalisation to zero at the end
        pos.linear.x = 0.0;
        pos.linear.y = 0.0;
        pos.linear.z = 0.0;
        pos.angular.x = 0.0;
        pos.angular.y = 0.0;
        pos.angular.z = 0.0;
        pub.publish(pos);
}

/** 
 * @brief Callback function for LaserScan data
 * @param data Message from /scan topic 
 * **/

void Walker::checkObstacle(const sensor_msgs::LaserScan::ConstPtr& data) {
    for(auto d : data->ranges) {
        if (d < 1.0) {
            checkCollision = true;
            break;
        }
        checkCollision = false;
    }
}

/** 
 * @brief Function implementing walker algorithm
 * @return void 
 * **/

void Walker::moveTurtle() {
    // Publishing Velocity on cmd_vel topic
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Subscribing to topic /scan and using checkObstacle 
    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",50, &Walker::checkObstacle, this);

    // Set loop rate 
    ros::Rate loopRate(10);


    while(ros::ok()) {
        if (checkCollision) {
            ROS_INFO_STREAM("Obstacle Detected! Turning ...");
            pos.linear.x = 0.0;
            pos.angular.z = 1.5;
        } else {
            ROS_INFO_STREAM("Moving ahead ...");
            pos.linear.x = 0.3;
            pos.angular.z = 0.0;
        }

        pub.publish(pos);
        ros::spinOnce();
        loopRate.sleep();


}
}
