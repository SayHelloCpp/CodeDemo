
#include <serial/serial.h>
#include <deque>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "ultrasonic/U_driver.h"

int main(int argc, char **argv) {
    ROS_INFO("[info] init begin");
    ros::init(argc, argv, "serial_port");

    sub_and_pub::U_driver u_driver;
    u_driver.Run();

    ros::spin();
    return 0;
}