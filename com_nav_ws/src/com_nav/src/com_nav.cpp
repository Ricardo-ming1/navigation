#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <queue>
#include <array>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include "CRC.h"

class SerialCommunicationNode : public rclcpp::Node {
public:
    SerialCommunicationNode() : Node("serial_communication_node"), fd_(-1) {
        if(!initializeSerial()){
            throw std::runtime_error("Serial port initialization failed");
        }
        //initializeSerial();  // 初始化串口
        /*subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SerialCommunicationNode::cmdVelCallback, this, std::placeholders::_1)
        );  // 订阅 /cmd_vel 话题 */
         subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,[this](const geometry_msgs::msg::Twist::SharedPtr msg) {cmdVelCallback(msg);});
    }

    ~SerialCommunicationNode() {
        if (fd_ >= 0) {
            close(fd_);  // 关闭串口
        }
    }

private:
    static constexpr size_t BUFFER_SIZE = 1024;  // 缓冲区大小
    static constexpr uint8_t FRAME_HEADER1 = 0x42;  // 帧头 1
    static constexpr uint8_t FRAME_HEADER2 = 0x52;  // 帧头 2
    static constexpr uint8_t COMMAND_CODE = 0xCD;   // 命令码

    int fd_;  // 串口文件描述符
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;  // 订阅器

    std::string findSerialPort() {
    // 动态检测所有ACM设备
    for(int i=0; ; ++i){
        std::string port = "/dev/ttyACM" + std::to_string(i);
        if(access(port.c_str(), F_OK) != 0)  break;        
        // 验证确实是串口设备
        struct stat st;
        if(stat(port.c_str(), &st) == 0 && S_ISCHR(st.st_mode)){
            return port;
        }
        }
        return "";
    }

    bool checkAndModifyPermissions(const std::string& port) {
            RCLCPP_WARN(get_logger(), "Attempting to modify permissions for %s", port.c_str());
            
            // 添加密码提示
            std::cout << "[sudo] password required for modifying device permissions: ";
            // 立即刷新输出缓冲区
            fflush(stdout);

            // 构建完整的权限修改命令
            std::string command = "sudo chmod 666 " + port + " 2>&1";


            RCLCPP_INFO(get_logger(), "Executing: %s", command.c_str());
            
            // 使用popen获取更详细的输出
            FILE* pipe = popen(command.c_str(), "w");
            if (!pipe) {
                RCLCPP_ERROR(get_logger(), "popen() failed!");
                return false;
            }
            
            // 如果需要密码，这里会阻塞直到用户输入
            int ret = pclose(pipe);

            if (WIFEXITED(ret)) {
                int exit_code = WEXITSTATUS(ret);  // 获取退出状态码
                if (exit_code != 0) {
                    RCLCPP_ERROR(get_logger(), "Permission change failed with code: %d", exit_code);
                    return false;
                }
            }
            /*
            // 添加udev规则建议
            RCLCPP_WARN(get_logger(),
                "For permanent access, create udev rule:\n"
                "echo 'SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"[VID]\", "
                "ATTRS{idProduct}==\"[PID]\", MODE=\"0660\", GROUP=\"dialout\"' | "
                "sudo tee /etc/udev/rules.d/99-serial.rules\n"
                "sudo udevadm control --reload-rules");
            */
        return true;
    }

    bool configureSerialPort() {
        termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "Error getting termios: %s", strerror(errno));
            return false;
        }

        // 波特率设置
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // 控制模式设置
        tty.c_cflag &= ~PARENB;   // 禁用奇偶校验
        tty.c_cflag &= ~CSTOPB;   // 1位停止位
        tty.c_cflag &= ~CSIZE;    // 清除数据位掩码
        tty.c_cflag |= CS8;       // 8位数据位
        tty.c_cflag &= ~CRTSCTS;  // 禁用硬件流控
        tty.c_cflag |= CREAD | CLOCAL;  // 启用接收，忽略调制解调器状态

        // 本地模式设置
        tty.c_lflag &= ~ICANON;   // 禁用规范模式
        tty.c_lflag &= ~ECHO;     // 禁用回显
        tty.c_lflag &= ~ECHOE;    // 禁用擦除
        tty.c_lflag &= ~ECHONL;   // 禁用换行回显
        tty.c_lflag &= ~ISIG;     // 禁用信号

        // 输入模式设置
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 禁用软件流控
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        // 输出模式设置
        tty.c_oflag &= ~OPOST;   // 原始输出模式
        tty.c_oflag &= ~ONLCR;   // 禁用换行转换

        // 超时设置
        tty.c_cc[VTIME] = 10;    // 读取超时（单位：0.1秒）
        tty.c_cc[VMIN] = 0;      // 最小读取字节数

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "Error setting termios: %s", strerror(errno));
            return false;
        }

        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    bool initializeSerial() {
        // 步骤1：自动检测设备
        std::string port = findSerialPort();
        if (port.empty()) {
            RCLCPP_ERROR(get_logger(), "No compatible serial device found!");
            return false;
        }
        //显示查找的串口
        RCLCPP_INFO(get_logger(), "Detected device: %s", port.c_str());

        // 步骤2：尝试打开设备
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            if (errno == EACCES) {
                if (!checkAndModifyPermissions(port)) {
                    RCLCPP_ERROR(get_logger(), "Permission modification failed");
                    return false;
                }

                // 重新尝试打开
                fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            }
            
            if (fd_ < 0) {
                RCLCPP_ERROR(get_logger(), "Open failed: %s", strerror(errno));
                return false;
            }
        }

        // 步骤3：配置串口参数
        if (!configureSerialPort()) {
            close(fd_);
            fd_ = -1;
            return false;
        }

        RCLCPP_INFO(get_logger(), "Serial port initialized: %s @ 115200 baud", port.c_str());
        return true;
    }
    /*
        //std::string port = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM2");  // 串口设备路径
        int baudrate = this->declare_parameter<int>("baudrate", 115200);  // 波特率

        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);  // 打开串口
        if (fd_ < 0) {
            if(errno == EACCES){
                RCLCPP_WARN(get_logger(), "Permission denied, attempting to modify permissions...");
                std::string command = "sudo chmod 666 " + port + " 2>&1";
                RCLCPP_INFO(get_logger(), "Executing: %s", command.c_str());
                int ret = system(command.c_str());

                if(ret!=0){
                    RCLCPP_ERROR(get_logger(), "Permission change failed. Error code: %d", ret);
                    RCLCPP_ERROR(get_logger(), "Please enter password if prompted or run with sudo");

                    return false;
                }
                //再次尝试
                fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            }
            if(fd_ < 0){
                RCLCPP_ERROR(get_logger(), "Failed to open port %s", port.c_str());  // 打开失败，记录错误
                return false;
            }           
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));  // 初始化 termios 结构体

        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "Failed to get serial attributes");  // 获取串口属性失败
            close(fd_);
            fd_ = -1;
            return;
        }

        cfsetospeed(&tty, B115200);  // 设置输出波特率
        cfsetispeed(&tty, B115200);  // 设置输入波特率

        tty.c_cflag &= ~CSIZE;            // 清除数据位掩码
        tty.c_cflag |= CS8;               // 设置数据位为 8
        tty.c_cflag &= ~PARENB;           // 禁用奇偶校验
        tty.c_cflag &= ~CSTOPB;           // 设置停止位为 1
        tty.c_cflag &= ~CRTSCTS;          // 禁用硬件流控制

        tty.c_lflag &= ~ICANON;           // 禁用规范模式
        tty.c_lflag &= ~ECHO;             // 禁用回显
        tty.c_lflag &= ~ISIG;             // 禁用信号处理
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 禁用软件流控制
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);  // 禁用输入处理
        tty.c_oflag &= ~OPOST;            // 禁用输出处理

        tty.c_cc[VMIN] = 0;               // 最小读取字节数
        tty.c_cc[VTIME] = 1;              // 读取超时时间

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "Failed to set serial attributes");  // 设置串口属性失败
            close(fd_);
            fd_ = -1;
            return;
        }

        tcflush(fd_, TCIOFLUSH);  // 清空输入输出缓冲区
        RCLCPP_INFO(get_logger(), "Serial initialized: %s @ %d", port.c_str(), baudrate);  // 记录初始化成功
    }
    */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 将 Twist 消息转换为字节流
        std::array<uint8_t, 30> tx_data{};

        tx_data[0] = FRAME_HEADER1;  // 帧头 1
        tx_data[1] = FRAME_HEADER2;  // 帧头 2
        tx_data[2] = COMMAND_CODE;   // 命令码
        tx_data[3] = 0x18;        // 数据长度（ 字节）

        // 填充线速度和角速度
        memcpy(&tx_data[4], &msg->linear.x, sizeof(double));     //    4 5 6 7 8 9 10 11 
        memcpy(&tx_data[13], &msg->linear.y, sizeof(double));     //    13 14 15 16 17 18 19 20
        memcpy(&tx_data[21], &msg->angular.z, sizeof(double));   //    21 22 23 24 25 26 27 28

        // 计算 CRC 校验
        tx_data[29] = CRC8_Check_Sum(tx_data.data(), 29);

        // 发送数据
        ssize_t written = write(fd_, tx_data.data(), tx_data.size());

        if (written == static_cast<ssize_t>(tx_data.size())) {
            RCLCPP_INFO(get_logger(), "TX: linear.x=%.2lf linear.y=%.2lf angular.z=%.2lf", 
                         msg->linear.x, msg->linear.y, msg->angular.z);
            /*RCLCPP_INFO(get_logger(), "TX Data:");
            for(size_t i=0;i<tx_data.size();++i){
                RCLCPP_INFO(get_logger(), "%02X ", tx_data[i]);
            }*/
        }else {
            RCLCPP_ERROR(get_logger(), "Failed to send data");
        }
    }
};
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);  // 初始化 ROS 2

    try{
        auto node = std::make_shared<SerialCommunicationNode>();  // 创建节点
        rclcpp::spin(node);  // 运行节点
    } catch (const std::exception& e){
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Initialization failed: %s", e.what());
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();  // 关闭 ROS 2
    return EXIT_SUCCESS;
}