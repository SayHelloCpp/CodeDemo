#include <ros/ros.h>
//串口库
#include <serial/serial.h>
//这些是ros中的消息类型
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <deque>
#include <vector>
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf/transform_datatypes.h"

using namespace std;
//声明串口
serial::Serial ser;

//定义变量
float carbondioxide;  //二氧化碳
float formaldehyde;   //甲醛
float TVOC;           // TVOC不知道是啥
float PMtf;           // PM2.5 two fives
float PMten;          // PM10
float PMone;          // PM1.0
// float temperature;//温度
// float humidity;//湿度

float so_concentration;  // so浓度

float hs_concentration;  // hs浓度

float temperature;  //温度
float humidity;     //湿度

float decibel;  //分贝
//这是根据通信协议确定的（图1）
unsigned char all_data[4] = {0xFF, 0xA5, 0x10, 0xB4};

int main(int argc, char** argv) {
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;
    // Publisher

    //订阅主题，并配置回调函数
    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题 这里发布了很多个，是因为之前这些传感器是单独的，不让别人改订阅
    // ros中有许多消息类型，选择合适的进行发布，在订阅时需要知道消息类型的（我这里不太好，完成功能）
    ros::Publisher noise_pub      = nh.advertise<std_msgs::String>("/noise", 1);
    ros::Publisher temhum_pub     = nh.advertise<std_msgs::Float64MultiArray>("/temhum", 1);
    ros::Publisher six_sensor_pub = nh.advertise<std_msgs::Float64MultiArray>("/mp_all", 1);  //多合一，6个值
    ros::Publisher hs_pub         = nh.advertise<std_msgs::String>("/hsulfide", 1);
    ros::Publisher so_pub         = nh.advertise<std_msgs::String>("/sdioxide", 1);

    try {
        //设置串口属性，并打开串口
        //这里的/dev/ttyUSB0是usb口在linux下读出来的，也有可能是其他的，使用com口的话。则会是/dev/ttyS*这样的形式
        ser.setPort("/dev/ttyUSB0");
        //波特率是通信协议制订的
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //指定循环的频率
    ros::Rate loop_rate(1);
    uint8_t buffer[1024];
    uint16_t cnt = 0;
    // deque 双端队列
    std::deque<uint8_t> data_deque;
    while (ros::ok()) {
        //下发数据，就是往串口发送指令
        ser.write(all_data, 4);
        size_t n = ser.available();
        // printf("d:%d\n",n);
        if (n != 0) {
            //读出数据
            n = ser.read(&buffer[cnt], n);
            //这一段可以不用
            for (int i = 0; i < n; i++) {
                // 16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << "  ";
                //数据推入队列，push_back 从后往前
                data_deque.push_back(uint8_t((buffer[i])));
            }
            std::cout << std::endl;
            //开始做数据解析，对读上来的数据解析，第几位是什么数据
            while (data_deque.size() > 37) {
                //数据解析 根据协议可以知道是采用 和校验的方式 帧头为0xFF
                //计算和，将和接受到最后一帧数据进行对比
                int sum = 0;
                for (int i = 0; i < 37; i++) {
                    sum += uint8_t(data_deque[i]);
                    //这是取数据的低八位，
                    sum = sum & 0xFF;
                }
                //这一段是数据解析，做帧头与和校验，保证数据准确
                if (data_deque[0] == 0xFF) {
                    // printf("ok");
                    if (sum == data_deque[37]) {
                        decibel = float(data_deque[3] * 256 + data_deque[4]);
                        decibel = decibel / 10;

                        humidity = float(data_deque[6] * 256 + data_deque[7]);
                        humidity = humidity / 10;

                        temperature = float(data_deque[8] * 256 + data_deque[9]);
                        temperature = temperature / 10;

                        PMtf = float(data_deque[15] * 256 + data_deque[16]);

                        carbondioxide = float(data_deque[17] * 256 + data_deque[18]);

                        TVOC = float(data_deque[19] * 256 + data_deque[20]);
                        TVOC = TVOC / 833.33;

                        PMone = float(data_deque[21] * 256 + data_deque[22]);

                        PMten = float(data_deque[23] * 256 + data_deque[24]);

                        formaldehyde = float(data_deque[25] * 256 + data_deque[26]);

                        so_concentration = float(data_deque[29] * 256 + data_deque[30]);

                        hs_concentration = float(data_deque[34] * 256 + data_deque[35]);
                    }
                    data_deque.pop_front();
                }
                data_deque.pop_front();
            }
        }
        // zaoyin
        std_msgs::String noise_msg;
        std::stringstream noise;
        noise << decibel;
        noise_msg.data = noise.str();
        noise_pub.publish(noise_msg);

        // wenshidu
        std::vector<double> temhum_;
        temhum_.push_back(temperature);
        temhum_.push_back(humidity);

        std_msgs::String tem;
        std::stringstream tem1;
        tem1 << temperature;
        tem.data = tem1.str();

        std_msgs::String hum;
        std::stringstream hum1;
        hum1 << humidity;
        hum.data = hum1.str();

        std_msgs::Float64MultiArray temhum_and;
        temhum_and.data = temhum_;
        temhum_pub.publish(temhum_and);

        // qiheyi 以数组的形式发布
        std::vector<double> six_;
        six_.push_back(PMtf);
        six_.push_back(carbondioxide);
        six_.push_back(TVOC);
        six_.push_back(PMone);
        six_.push_back(PMten);
        six_.push_back(formaldehyde);

        std_msgs::String msg1;
        std::stringstream ss1;
        ss1 << PMtf;
        msg1.data = ss1.str();

        std_msgs::String msg2;
        std::stringstream ss2;
        ss2 << carbondioxide;
        msg2.data = ss2.str();

        std_msgs::String msg3;
        std::stringstream ss3;
        ss3 << TVOC;
        msg3.data = ss3.str();

        std_msgs::String msg4;
        std::stringstream ss4;
        ss4 << PMone;
        msg4.data = ss4.str();

        std_msgs::String msg5;
        std::stringstream ss5;
        ss5 << PMten;
        msg5.data = ss5.str();

        std_msgs::String msg6;
        std::stringstream ss6;
        ss6 << formaldehyde;
        msg6.data = ss6.str();

        std_msgs::Float64MultiArray six_and;
        six_and.data = six_;
        six_sensor_pub.publish(six_and);

        // so
        std_msgs::String so_msg;
        std::stringstream so;
        so << so_concentration;
        so_msg.data = so.str();
        so_pub.publish(so_msg);
        // hs
        std_msgs::String hs_msg;
        std::stringstream hs;
        hs << hs_concentration;
        hs_msg.data = hs.str();
        hs_pub.publish(hs_msg);

        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();
    }
}
