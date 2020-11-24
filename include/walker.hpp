#pragma once
/**
 *  Copyright 2020 Nidhi Bhojak
 *  @file walker.hpp
 *  @author Nidhi Bhojak
 *  @date 11/20/2020
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
 *  Declaration of the walker class   
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

/** @brief Walker Class
 * 
 *
 * Class contains variable to read from sensor data and functions to facilitate
 * turning when required
 * **/

class Walker {
 private:
        
        // ROS NodeHandle Initialization
        ros::NodeHandle nh;
        //ROS Publisher 
        ros::Publisher pub;
        //ROS Subscriber
        ros::Subscriber sub;
        // Variable for publishing velocity
        geometry_msgs::Twist pos;
        // Boolean variable to check collision 
        bool checkCollision;
 public: 
        /**
         * @brief Walker Class Constructor 
         * @param None
         *  **/
        Walker();
        /**
         * @brief Walker Class Destructor
         * @param None 
         *  **/
        ~Walker();
        /** 
         * @brief Callback function for LaserScan 
         * @param data Single scan from a planar range finder
         * @return void 
         * **/
        void checkObstacle(const sensor_msgs::LaserScan::ConstPtr& data);  
        /** 
         * @brief Function to implement the walker algorithm
         * @return void 
         * **/ 
        void moveTurtle();    
};

