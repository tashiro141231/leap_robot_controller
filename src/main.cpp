#include <stdio.h>
#include <stdint.h>
#include <ros/ros.h>
#include <leap_motion/leapros.h>
#include <leap_motion/leap.h>

#include "cv_bridge/cv_bridge.h"
// #include "opencv2/core/core.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/opencv.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// #include "opencv2/core.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/opencv.hpp"

double hand_pos[3] = {0,0,0};

void LeapCallback(leap_motion::leapros data) {

  hand_pos[0] = data.palmpos.x/200;
  hand_pos[1] = data.palmpos.y/200;
  hand_pos[2] = data.palmpos.z/200;
  
  if(data.sphere_radius >0 && data.sphere_radius < 45) {
    std::cout << "Grabbing" << std::endl;
    // std::cout << "ID = " << data.header.frame_id << std::endl;
    // std::cout << "x = " << data.palmpos.x << " y = " << data.palmpos.y << " z = " << data.palmpos.z << std::endl;
    std::cout << "x = " << data.palmpos.x << " z = " << data.palmpos.z << std::endl;
  }
  else {
    std::cout << data.sphere_radius << " [mm]" << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "leap_robot_controller_node");
  ros::NodeHandle handle;
  ros::Subscriber sub = handle.subscribe("leapmotion/data", 100, LeapCallback);
  ros::Rate rate(50);
  // cv::Mat img = cv::Mat::zeros(200, 200, CV_8UC3);
  // cv::String window = "hand position";
  // namedWindow("hand_pos", cv::WINDOW_AUTOSIZE);

  while(ros::ok()) {
    ros::spinOnce();

    // cv::circle(img, cv::Point(hand_pos[0], hand_pos[2]), 80, cv::Scalar(0,200,0), 2,4); 
    // cv::imshow("hand_pos", img);
    // cv::imshow("Original", img);
    rate.sleep();
  }
}

