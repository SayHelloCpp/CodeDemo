#include "ultrasonic/U_driver.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

namespace sub_and_pub {

U_driver::U_driver() {
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    // 设置要打开的串口名称
    sp_.setPort("/dev/ttyUSB1");
    //设置串口通信的波特率
    sp_.setBaudrate(115200);
    //串口设置timeout
    sp_.setTimeout(to);
    //注册发布器
    pub_ = nh_.advertise<sensor_msgs::Range>("/ultrasonic", 1000);
    //有效数据的长度
    valid_data_len_ = 71;
    data_buffer_.clear();
    ROS_INFO_STREAM("[U_driver]: Init over");
}
U_driver::~U_driver() {}

bool U_driver::GetDataFromUsb(std::deque<float>& range_data_vec, std::deque<float>& confidence_data_vec) {
    setlocale(LC_ALL, "");
    range_data_vec.clear();
    confidence_data_vec.clear();
    // 0. 获取缓冲区内的字节数
    size_t n = sp_.available();
    // 1. 若字节数为非零，则成功获取到串口数据
    if (n != 0) {
        // 2. 读出数据
        std::string current_buffer = sp_.read(n);
        std ::cout << "n: " << n << std::endl;
        data_buffer_.insert(data_buffer_.size(), current_buffer);
        for (int i = 0; i < n; i++) {
            // 16进制的方式打印到屏幕
            std::cout << std::hex << (current_buffer[i] & 0xff) << "  ";
        }

        // 3. 开始做数据解析，对读到的数据解析，第几位是什么数据
        //由于该while不为死循环，为了加快数据的发布，暂不加sleep函数
        ros::Rate loop_rate(10);
        while (!data_buffer_.empty()) {
            // 4. 提取帧头和命令字的索引
            int head         = data_buffer_.find_first_not_of((char)(0x5A));
            int commond_word = data_buffer_.find_first_not_of((char)(0xA0));
            // ROS_INFO_STREAM("head: " << head << ",command_word: " << commond_word << " ");
            // 5.判断命令字是否在帧头的后一位,若成立，则将帧头前面的数据去掉，保留帧头及其之后的数据，否则删除帧头和命令字，继续寻找下一个帧头和命令字
            if (head >= 0 && commond_word == (head + 1)) {
                // ROS_INFO_STREAM("header+command_word = 0x5AA0");
                data_buffer_ =
                    data_buffer_.substr(head, data_buffer_.size());  //!取出定长的数据在head之后的后 长度为72的数据
            } else {
                data_buffer_ = data_buffer_.substr(commond_word + 1, data_buffer_.size());
                ROS_WARN_STREAM_DELAYED_THROTTLE(1, "[U_driver]: Head and commond word is invalid!");
                loop_rate.sleep();
                continue;
            }

            int data_len = data_buffer_.size();

            // 6. 判断剩余数据长度是否大于有效数据的长度,若成立，则校验数据是否符合通讯协议
            if (data_len >= valid_data_len_) {
                if (!DataCheckout(data_buffer_)) {
                    //校验未通过，则删除当前校验的数据，即一组有效数据的长度，继续寻找下一个帧头和命令字
                    data_buffer_ = data_buffer_.substr(valid_data_len_, data_len);
                    ROS_WARN_STREAM("[U_driver]: Data checkout failed");
                    loop_rate.sleep();
                    continue;
                }
                //校验通过，则解析出对应的测距和置信度数据
                ROS_WARN_STREAM("[U_driver]: Data checkout sucessed");
                for (int i = 0; i < valid_data_len_; i++) {
                    std::cout << std::hex << (data_buffer_[i] & 0xff) << "  ";
                }
                std::string range_data      = data_buffer_.substr(7, 8);
                std::string confidence_data = data_buffer_.substr(9, 10);
                float range                 = float(GetParseData(range_data)) / 1000.0;
                float confidence            = GetParseData(confidence_data);
                //将解析后的数据推入有效数据容器中
                range_data_vec.push_back(range);
                confidence_data_vec.push_back(confidence);
                //将使用过的有效数据删除，保留未使用的数据
                data_buffer_ = data_buffer_.substr(valid_data_len_, data_len);
            } else {
                //剩余长度小于有效数据的长度，不足以得到一组有效数据，则继续获取串口数据
                ROS_WARN_STREAM("[U_driver]: Data len is not enough");
                return true;
            }
        }
        return true;
    } else {
        ROS_ERROR_STREAM("[U_driver]: Can not read data");
        return false;
    }
}

void U_driver::Run() {
    try {
        //打开串口
        sp_.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return;
    }
    //判断串口是否打开成功
    if (sp_.isOpen()) {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    } else {
        return;
    }
    ros::Rate r(50);
    while (ros::ok()) {
        std::deque<float> range_data_vec;
        std::deque<float> confidence_data_vec;
        if (GetDataFromUsb(range_data_vec, confidence_data_vec)) {
            while (!range_data_vec.empty() && !confidence_data_vec.empty()) {
                Pubilisher(range_data_vec.front(), confidence_data_vec.front());
                range_data_vec.pop_front();
                confidence_data_vec.pop_front();
                ROS_INFO_STREAM_DELAYED_THROTTLE(1, "[U_driver]: Pub ultrasonic data success");
            }
        } else {
            ROS_WARN_STREAM("[U_driver]: Can not get ultrasonic data!");
        }
        r.sleep();
    }
}

bool U_driver::DataCheckout(const std::string& data) {
    //校验数据是否有效
    size_t len  = 72;
    uint8_t num = 0;
    for (int i = 0; i < len - 1; i++) {
        num += (data.at(i) & 0xff);
        // std::cout << std::hex << "num:" << (num & 0xff) << "   ";
    }
    // std::cout << std::hex << "第一个数据为：" << (data.at(0) & 0xff) << " ";
    num                = ~num;
    uint8_t check_data = (uint8_t)(data.at(len - 1) & 0xff);
    // ROS_WARN_STREAM("[U_driver]: Data Typing...");
    for (int i = 0; i < len; i++) {
        std::cout << std::hex << (data.at(i) & 0xff) << " ";
    }
    // ROS_WARN_STREAM("[U_driver]: Data Typing... Over");

    /*     std::cout << std::hex << "处理后的数据和取反为" << (num & 0xff) << std::endl;
        std::cout << std::hex << "帧尾即校验位值为 " << (check_data & 0xff) << std::endl;
     */
    if (num == check_data)
        return true;
    else
        return false;
}

int U_driver::GetParseData(const std::string& data) {
    //获取解析后的数据
    int16_t tmp = 0;
    if (data.size() != 2)
        return tmp;
    tmp = int16_t(((data[0] & 0xff) << 8) | ((data[1] & 0xff) << 0));
    return int(tmp);
}

void U_driver::Pubilisher(const float& range, const float& confidence) {
    sensor_msgs::Range msg;
    msg.header.frame_id = "ultrasonic_link";
    msg.header.stamp    = ros::Time::now();
    msg.radiation_type  = sensor_msgs::Range::ULTRASOUND;
    msg.min_range       = 0.01;
    msg.max_range       = 1.0;
    msg.field_of_view   = confidence;  //使用视场角这个参数来表示confidence
    msg.range           = range;
    pub_.publish(msg);
}
}  // namespace sub_and_pub
