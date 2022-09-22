

// #ifndef SRC_UART_TUTORIAL_INCLUDE_GRIPPER_REC_H_
// #define SRC_UART_TUTORIAL_INCLUDE_GRIPPER_REC_H_

#include <ros/ros.h>  // 包含ROS的头文件
#include <sensor_msgs/Range.h>
#include <serial/serial.h>
#include <deque>
#include <iostream>
#include <vector>

namespace sub_and_pub {

class U_driver {
   public:
    U_driver();
    ~U_driver();
    void Run();

   private:
    void Pubilisher(const float& range, const float& confidence);
    bool GetDataFromUsb(std::deque<float> &range_data_vec, std::deque<float>& confidence_data_vec);
    bool DataCheckout(const std::string& data);
    int GetParseData(const std::string& data);

   private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    serial::Serial sp_;

    std::string data_buffer_;
    int valid_data_len_;
};
}  // namespace sub_and_pub

// #endif  // SRC_UART_TUTORIAL_INCLUDE_GRIPPER_REC_H_