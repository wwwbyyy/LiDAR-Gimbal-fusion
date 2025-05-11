#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <utility>
#include <cstdint>
#include <signal.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <queue>
#include <chrono>
#include <iostream>
#include "GimbalCommand.h"

typedef enum command
{
    STOP      = 0x00,  // 停止
    UP        = 0x08,  // 上
    DOWN      = 0x10,  // 下
    LEFT      = 0x04,  // 左
    RIGHT     = 0x02,  // 右
    UPLEFT    = 0x0C,  // 上左, 12
    UPRIGHT   = 0x0A,  // 上右, 10
    DOWNLEFT  = 0x14,  // 下左, 20
    DOWNRIGHT = 0x12,  // 下右, 18
    PAN       = 0x4B,  // 水平旋转至指定角度, 75
    TILT      = 0x4D,  // 俯仰旋转至指定角度, 77
    ASKPAN    = 0x51,  // 查询水平位置, 81
    ASKTILT   = 0x53   // 查询俯仰位置, 83
};

typedef enum serial_port_status
{
    WRITE = 0,
    READ = 1
};

// 定义全局变量
ros::Publisher msg_pub;
ros::Publisher pan_pub;
ros::Publisher tilt_pub;
ros::Subscriber cmd_sub;
double start_time;
// std::ofstream outputFilepan("feedback_data_pan.csv", std::ios::app);
// std::ofstream outputFiletilt("feedback_data_tilt.csv", std::ios::app);
const size_t read_buffer_size = 7; // PELCO-D 协议反馈数据长度

const float tilt_zero_deg = -3; // the realistic zero degree of tilt
float pan_angle = 0;
float tilt_angle = 0;

// 串口和互斥锁
serial::Serial serial_port;
std::mutex serial_mutex;
int serial_port_status = READ;

// 控制指令队列
std::queue<std::vector<uint8_t>> command_queue;
std::mutex queue_mutex;

// 将角度值映射到0-360度的范围
float normalize_angle_pan(float angle) 
{
    float normalized_angle = fmod(angle, 360.0);
    if (normalized_angle < 0) {
        normalized_angle += 360.0;  // 处理负角度
    }
    return normalized_angle;
}

float normalize_angle_tilt(float angle) 
{
    // 0水平，向下0-90，向上359-270，留5度限位
    float normalized_angle = std::max(std::min(angle, 85.0f), -85.0f);
    normalized_angle = normalized_angle + tilt_zero_deg;
    normalized_angle = std::max(std::min(normalized_angle, 85.0f), -85.0f);
    if (normalized_angle < 0) {
        normalized_angle = 360.0 + normalized_angle;
    }
    // printf("tilt: %f, normalized tilt: %f\n", angle, normalized_angle);
    return normalized_angle;
}

float reverse_normalize_angle_tilt(float angle)
{
    float normalized_angle = angle - tilt_zero_deg;
    // if (normalized_angle < 0)
    // {
    //     normalized_angle = 360.0 + normalized_angle;
    // }
    return normalized_angle;
}

// 将位置（float类型，角度制）映射为PELCO-D的位置值，范围为0x0000到0xFFFF
std::pair<uint8_t, uint8_t> position_to_data(uint8_t cmd, float position) 
{
    uint16_t pos; 
    if (cmd == PAN)
    {
        position = normalize_angle_pan(-position);  // 将角度映射到0-360度范围内
        pos = static_cast<uint16_t>(position * 100.0);
    }
    else if (cmd == TILT)
    {
        position = normalize_angle_tilt(position);
        pos = static_cast<uint16_t>(position * 100.0);
    }
    uint8_t data1 = static_cast<uint8_t>((pos >> 8) & 0xFF); // 高位字节
    uint8_t data2 = static_cast<uint8_t>(pos & 0xFF);        // 低位字节
    return {data1, data2};
}

float data_to_position(uint8_t cmd, uint8_t data0, uint8_t data1)
{
    uint16_t pos = data0;
    pos <<= 8;
    pos += data1;
    float position = pos / 100.0;
    // printf("cmd = %d\n", cmd);
    if (cmd == ASKPAN)
    {
        position = 360 - position;
    }
    if (cmd == ASKTILT)
    {
        if (position > 180)
            position = position - 360;
    }
    return position;
}

// 将速度（float类型）映射为PELCO-D的速度值
std::pair<uint8_t, uint8_t> speed_to_data(float speedpan, float speedtilt)
{
    // 假设速度范围是0到100，对应的数据范围是0到0x3F（PELCO-D速度范围0x00 - 0x3F, 0-63）
    uint8_t data1 = static_cast<uint8_t>(std::round(std::min(std::max(speedpan, 0.0f), 63.0f)));
    uint8_t data2 = static_cast<uint8_t>(std::round(std::min(std::max(speedtilt, 0.0f), 63.0f)));
    return {data1, data2};
}


// PELCO-D协议命令格式
std::vector<uint8_t> pelco_d_command(uint8_t cmd, float data) 
{
    std::vector<uint8_t> command(7);
    command[0] = 0xFF;              // 同步字节
    command[1] = 0x01;              // 地址字节
    command[2] = 0x00;              // 命令字节1
    command[3] = static_cast<uint8_t>(cmd);

    // 根据指令判断data是速度还是位置
    std::pair<uint8_t, uint8_t> data_bytes;
    switch (cmd)
    {
        case PAN:
            data_bytes = position_to_data(cmd, data);
            break;
        case TILT:
            data_bytes = position_to_data(cmd, data);
            break;
        case UP:
        case DOWN:
        case LEFT:
        case RIGHT:
            data_bytes = speed_to_data(data, data);
            break;
        case ASKPAN:
            data_bytes.first  = 0x0;
            data_bytes.second = 0x0;
            break;
        case ASKTILT:
            data_bytes.first  = 0x0;
            data_bytes.second = 0x0;
            break;
        default:
            command[3] = 0x00;
            data_bytes.first  = 0x0;
            data_bytes.second = 0x0;
            break;
    }
    // 将解析出的data1和data2存入指令
    command[4] = data_bytes.first;  // data1
    command[5] = data_bytes.second; // data2
    command[6] = (command[1] + command[2] + command[3] + command[4] + command[5]) % 256;  // 校验和
    return command;
}

// 数据校验函数
bool check_serial_data(const std::vector<uint8_t>& data) {
    if (data.size() == read_buffer_size) {
        if (data[0] == 0xFF && data[1] == 0x01 && (data[3] == 0x59 || data[3] == 0x5B)) { // 检查同步字节和地址字节
            if (data[6] == (data[1] + data[2] + data[3] + data[4] + data[5]) % 256) {
                return true;
            }
        }
    }
    return false;
}

// 读取角度函数
void read_angles() {
    while (ros::ok()) {
        double current_time_pan = start_time, current_time_tilt = start_time;
        std::vector<uint8_t> pan_feedback(read_buffer_size);
        std::vector<uint8_t> tilt_feedback(read_buffer_size);
        if (serial_port_status == READ) {
            // 读取水平角度
            {
                std::lock_guard<std::mutex> lock(serial_mutex);
                std::vector<uint8_t> pan_command = pelco_d_command(ASKPAN, 0.0);
                serial_port.write(pan_command);
                // double current_time_pan = ros::Time::now().toSec() - start_time;
                current_time_pan = ros::Time::now().toSec();
                size_t pan_bytes_read = serial_port.read(pan_feedback.data(), pan_feedback.size());
                pan_feedback.resize(pan_bytes_read);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            {
                std::lock_guard<std::mutex> lock(serial_mutex);
                // 读取垂直角度
                std::vector<uint8_t> tilt_command = pelco_d_command(ASKTILT, 0.0);
                serial_port.write(tilt_command);
                // double current_time_tilt = ros::Time::now().toSec() - start_time;
                current_time_tilt = ros::Time::now().toSec();
                size_t tilt_bytes_read = serial_port.read(tilt_feedback.data(), tilt_feedback.size());
                tilt_feedback.resize(tilt_bytes_read);
            }


            // 处理反馈数据
            if (check_serial_data(pan_feedback)) {
                pan_angle = data_to_position(pan_feedback[3] - 0x08, pan_feedback[4], pan_feedback[5]);
                std_msgs::Float64MultiArray pan_msg;
                pan_msg.data.push_back(current_time_pan);
                pan_msg.data.push_back(pan_angle);
                pan_pub.publish(pan_msg);
                // outputFilepan << current_time_pan - start_time << "\t" << -pan_angle << std::endl;
            }

            if (check_serial_data(tilt_feedback)) {
                tilt_angle = data_to_position(tilt_feedback[3] - 0x08, tilt_feedback[4], tilt_feedback[5]);
                std_msgs::Float64MultiArray tilt_msg;
                tilt_msg.data.push_back(current_time_tilt);
                tilt_msg.data.push_back(reverse_normalize_angle_tilt(tilt_angle));
                tilt_pub.publish(tilt_msg);
                // outputFiletilt << current_time_tilt - start_time << "\t" << reverse_normalize_angle_tilt(tilt_angle) << std::endl;
            }
        }
        // 25hz
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


// 鲁棒发送命令函数
void robust_send_command(const std::vector<uint8_t>& command, int timeout_ms = 100) {
    auto start_time = std::chrono::steady_clock::now();
    float pre_pan_angle = pan_angle, pre_tilt_angle = tilt_angle;

    while (ros::ok()) {
        std::vector<uint8_t> feedback_data(read_buffer_size);
        serial_port_status = WRITE;
        {
            std::lock_guard<std::mutex> lock(serial_mutex);
            serial_port.write(command);
        }
        // 等待一段时间再切换READ从而不会立即读取角度
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        serial_port_status = READ;

        // 检查是否超时
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() >= timeout_ms) {
            // ROS_WARN("Command execution timeout!");
            // return false;
            break;
        }
        // // 控制指令的精度是0.01
        // if (std::abs(pre_pan_angle - pan_angle) > 9e-3 || std::abs(pre_tilt_angle - tilt_angle) > 9e-3) {
        //     return true;
        // }
        // pre_pan_angle = pan_angle;
        // pre_tilt_angle = tilt_angle;
    }
}

// 处理控制指令函数
void process_commands() {
    // 自动读取命令（from rostopic）
    while (ros::ok()) {
        if (!command_queue.empty()) {
            std::vector<uint8_t> command;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                command = command_queue.front();
                command_queue.pop();
            }
            // 鲁棒发送命令
            robust_send_command(command);
        }
        // 等待一段时间再检查队列
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void gimbalCmdCallback(cyber_msgs::GimbalCommandConstPtr msg) {
    std::cout << "here" << std::endl;
    std::vector<uint8_t> command = pelco_d_command(msg->cmd, msg->data);
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        command_queue.push(command);
    }
}

// void gimbal_pan_callback(std_msgs::Float64MultiArrayConstPtr msg)
// {
//     std::cout << "here" << std::endl;

// }

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pelco_node");
    ros::NodeHandle nh;

    //signal(SIGINT, signalHandler);

    // 配置串口
    serial_port.setPort("/dev/ttyUSB0");  // 修改为实际的设备名
    serial_port.setBaudrate(38400);
    serial::Timeout to = serial::Timeout::simpleTimeout(50);
    serial_port.setTimeout(to);

    try {
        serial_port.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port!");
        return -1;
    }

    if (serial_port.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    msg_pub = nh.advertise<std_msgs::UInt8MultiArray>("/pelco_msg", 1);
    pan_pub = nh.advertise<std_msgs::Float64MultiArray>("/pan", 1);
    tilt_pub = nh.advertise<std_msgs::Float64MultiArray>("/tilt", 1);
    cmd_sub = nh.subscribe("/gimbal_cmd", 100, gimbalCmdCallback);
    // ros::Subscriber pan_sub = nh.subscribe("/pan", 100, gimbal_pan_callback);

    start_time = ros::Time::now().toSec();

    std::vector<uint8_t> command;
    command.resize(7);

    // 复位
    command = pelco_d_command(TILT, -20);
    serial_port.write(command);
    ros::Duration(0.5).sleep();
    command = pelco_d_command(PAN, -50);
    serial_port.write(command);
    ros::Duration(0.5).sleep();
    ROS_INFO("Reset.");

    // 测试命令
    // command_queue.push(pelco_d_command(RIGHT, 50));


    // 启动读取角度线程
    std::thread read_thread(read_angles);

    // 启动处理控制指令线程
    std::thread command_thread(process_commands);

    // 等待线程结束
    read_thread.detach();
    command_thread.detach();


    // 手动读取命令
    uint8_t cmd;
    float data;
    // while (ros::ok()) {
    //     std::cout << "input command: " << std::endl;
    //     scanf("%d%f", &cmd, &data);
    //     // printf("%d %d\n", cmd, data);
    //     std::vector<uint8_t> command = pelco_d_command(cmd, data);
    //     {
    //         std::lock_guard<std::mutex> lock(queue_mutex);
    //         command_queue.push(command);
    //     }
    //     std::cout << "command added to the queue." << std::endl;
        // ros::spinOnce();
    // }
    ros::spin();

    // ros::AsyncSpinner spinner(5);
    // spinner.start();

    // Your code here

    // ros::waitForShutdown();

    //STOP
    ROS_INFO("Will stop.");
    serial_port.write(pelco_d_command(STOP, 0x0));

    ROS_INFO("Stopped, and will close serial.");
    serial_port.close();
    ROS_INFO("Serial closed.");
    return 0;
}
