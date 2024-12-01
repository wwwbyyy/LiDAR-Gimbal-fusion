#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <utility>
#include <cstdint>

#define read_buffer_size 7

typedef enum command
{
    STOP      = 0x00,  // 停止
    UP        = 0x08,  // 上
    DOWN      = 0x10,  // 下
    LEFT      = 0x04,  // 左
    RIGHT     = 0x02,  // 右
    UPLEFT    = 0x0C,  // 上左
    UPRIGHT   = 0x0A,  // 上右
    DOWNLEFT  = 0x14,  // 下左
    DOWNRIGHT = 0x12,  // 下右
    PAN       = 0x4B,  // 水平旋转至指定角度
    TILT      = 0x4D,  // 俯仰旋转至指定角度
    ASKPAN    = 0x51,  // 查询水平位置
    ASKTILT   = 0x53   // 查询俯仰位置
};

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
    if (normalized_angle < 0) {
        normalized_angle = 360.0 + normalized_angle;
    }
    // printf("tilt: %f, normalized tilt: %f\n", angle, normalized_angle);
    return normalized_angle;
}


// 将位置（float类型，角度制）映射为PELCO-D的位置值，范围为0x0000到0xFFFF
std::pair<uint8_t, uint8_t> position_to_data(uint8_t cmd, float position) 
{
    uint16_t pos; 
    if (cmd == PAN)
    {
        position = normalize_angle_pan(position);  // 将角度映射到0-360度范围内
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
    // std::cout << "Send: ";
    // for (auto byte : command) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    // }
    // std::cout << std::endl;
    return command;
}

ros::Publisher msg_pub;
ros::Publisher pan_pub;
ros::Publisher tilt_pub;

bool check_serial_data(std::vector<uint8_t> data)
{
    if (data.size() == read_buffer_size)
    {
        if (data[3] == 0x59 || data[3] == 0x5B) //角度反馈
        {
            if (data[6] == (data[1] + data[2] + data[3] + data[4] + data[5]) % 256)
                return true;
        }
    }
    return false;
}

double start_time;
// 从串口读取反馈数据
void read_from_serial_port(serial::Serial& serial_port)
{
    std::vector<uint8_t> feedback_data;
    feedback_data.resize(read_buffer_size + 1); // 缓冲区，过大会导致读取变慢

    // 读取数据直到超时或读取到一定字节数
    size_t bytes_read = serial_port.read(feedback_data.data(), feedback_data.size());
    feedback_data.resize(bytes_read);

    // 数据校验
    if (check_serial_data(feedback_data))
    {
        float position = data_to_position(feedback_data[3] - 0x08, feedback_data[4], feedback_data[5]);
        printf("position: %.2f deg\n", position);
        std_msgs::Float32MultiArray feedback_msg;
        double current_time = ros::Time::now().toSec() - start_time;
        feedback_msg.data.push_back((float)current_time);
        feedback_msg.data.push_back(position);
        if (feedback_data[3] - 0x08 == ASKPAN)
            pan_pub.publish(feedback_msg);
        else if (feedback_data[3] - 0x08 == ASKTILT)
            tilt_pub.publish(feedback_msg);
    }
    

    // std_msgs::UInt8MultiArray feedback_msg;

    // for (auto byte : feedback_data) {
    //     feedback_msg.data.push_back(byte);
    // }
    // msg_pub.publish(feedback_msg);
}


int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pelco_node");
    ros::NodeHandle nh;

    // 配置串口
    serial::Serial my_serial;
    my_serial.setPort("/dev/ttyUSB0");  // 修改为实际的设备名
    my_serial.setBaudrate(2400);
    serial::Timeout to = serial::Timeout::simpleTimeout(50);
    my_serial.setTimeout(to);

    try {
        my_serial.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port!");
        return -1;
    }

    if (my_serial.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    msg_pub = nh.advertise<std_msgs::UInt8MultiArray>("/pelco_msg", 1);
    pan_pub = nh.advertise<std_msgs::Float32MultiArray>("/pan", 1);
    tilt_pub = nh.advertise<std_msgs::Float32MultiArray>("/tilt", 1);

    start_time = ros::Time::now().toSec();

    // ros::Timer timer = nh.createTimer(ros::Duration(0.08), angle_timer_callback);

    ros::Rate loop_rate(15);
    int ctrl = 0;
    int cnt = 0;
    std::vector<uint8_t> command;
    command.resize(7);

    // 复位
    command = pelco_d_command(TILT, 10.0);
    my_serial.write(command);

    while (ros::ok()) {

        // 提示用户输入十六进制命令和数据
        std::string cmd_input;
        float data = 50.0;

        if (cnt < 50)
        {
            cnt++;
            loop_rate.sleep();
            continue;
        }

        // // 获取十六进制命令
        // std::cout << "请输入十六进制命令: ";
        // std::cin >> cmd_input;

        // // 转换为整型
        // // unsigned int cmd_hex = 0x51;
        // unsigned int cmd_hex;
        // std::stringstream ss;
        // ss << std::hex << cmd_input;
        // ss >> cmd_hex;

        // // 输入角度/数据
        // std::cout << "请输入位置或速度数据 (角度制，浮点数): ";
        // std::cin >> data;

        // // 将输入命令转换为枚举类型
        // command cmd = static_cast<command>(cmd_hex);

        // std::vector<uint8_t> commands = pelco_d_command(cmd_hex, data);  // 地址1，上移，速度32


        if (ctrl == 0)
        {
            command[0] = 0xFF;              // 同步字节
            command[1] = 0x01;              // 地址字节
            command[2] = 0x00;              // 命令字节1
            command[3] = RIGHT;
            command[4] = 0x3F;
            command[5] = 0x00;
            command[6] = 0x01 + RIGHT + 0x3F;
            ctrl = 1;
        }
        else
        {
            command[0] = 0xFF;              // 同步字节
            command[1] = 0x01;              // 地址字节
            command[2] = 0x00;              // 命令字节1
            command[3] = ASKPAN;
            command[4] = 0x00;
            command[5] = 0x00;
            command[6] = 0x01 + ASKPAN;
            // ctrl = 0;
        }

        my_serial.write(command);

        // 等待反馈并处理
        read_from_serial_port(my_serial);

        // ros::spinOnce();
        loop_rate.sleep();
    }

    //STOP
    command[0] = 0xFF;              // 同步字节
    command[1] = 0x01;              // 地址字节
    command[2] = 0x00;              // 命令字节1
    command[3] = STOP;
    command[4] = 0x00;
    command[5] = 0x00;
    command[6] = 0x01 + STOP;
    my_serial.write(command);

    my_serial.close();
    return 0;
}
