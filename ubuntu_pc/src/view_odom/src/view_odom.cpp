#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // ROSメッセージへの変換用
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <stdexcept>

using std::placeholders::_1;
using nav_msgs::msg::Odometry;

// --- [設定] UDP受信側の情報 ---
const int LISTEN_PORT = 5001;
const char* ODOM_FRAME_ID = "odom";
const char* BASE_FRAME_ID = "base_link";
// -----------------------------

// 受信するデータ構造体 (TurtleBot4送信ノードと厳密に一致)
struct OdomData {
    double x;               // X位置 (8 bytes)
    double y;               // Y位置 (8 bytes)
    double yaw;             // ヨー角 (8 bytes)
    double lin_vel_x;       // 線形速度X (8 bytes)
    double ang_vel_z;       // 角速度Z (8 bytes)
    uint32_t frame_id;      // フレームID (4 bytes)
}; // 合計: 44 bytes

class OdomViewNode : public rclcpp::Node
{
public:
    OdomViewNode()
        : Node("view_odom"), is_running_(true), udp_socket_fd_(-1) // ノード名を 'view_odom' に設定
    {
        // ROSパブリッシャーの初期化
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/view_odom/odom_fwd", 10);

        // UDPソケットの設定とバインド
        setup_udp_socket();

        // 受信スレッドの起動
        receiver_thread_ = std::thread(&OdomViewNode::udp_receiver_loop, this);

        RCLCPP_INFO(this->get_logger(), "Node 'view_odom' started. Listening on port %d.", LISTEN_PORT);
    }

    ~OdomViewNode() override
    {
        is_running_ = false;
        if (receiver_thread_.joinable()) {
            shutdown(udp_socket_fd_, SHUT_RDWR);
            receiver_thread_.join();
        }
        if (udp_socket_fd_ >= 0) {
            close(udp_socket_fd_);
        }
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::thread receiver_thread_;
    std::atomic<bool> is_running_;
    int udp_socket_fd_;

    void setup_udp_socket()
    {
        // ... (UDPソケット設定ロジックは省略) ...
        struct sockaddr_in serverAddr;
        
        if ((udp_socket_fd_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "socket() failed");
            throw std::runtime_error("Failed to create UDP socket.");
        }

        std::memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        serverAddr.sin_port = htons(LISTEN_PORT);

        if (bind(udp_socket_fd_, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            close(udp_socket_fd_);
            RCLCPP_FATAL(this->get_logger(), "bind() failed on port %d", LISTEN_PORT);
            throw std::runtime_error("Failed to bind UDP socket.");
        }
    }

    void udp_receiver_loop()
    {
        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);
        OdomData receivedData;

        while (is_running_)
        {
            ssize_t recvSize = recvfrom(
                udp_socket_fd_, 
                (char*)&receivedData, 
                sizeof(OdomData), 
                0, 
                (struct sockaddr*)&clientAddr, 
                &clientAddrLen
            );

            if (recvSize < 0) {
                if (!is_running_) break;
                RCLCPP_ERROR(this->get_logger(), "recvfrom() failed in loop.");
                continue;
            }

            if (recvSize == sizeof(OdomData)) {
                // ROSのメインスレッドで実行される publish_odom_message を呼び出す
                publish_odom_message(receivedData);
            } else {
                RCLCPP_WARN(this->get_logger(), "Received incomplete data: %zd bytes.", recvSize);
            }
        }
        RCLCPP_INFO(this->get_logger(), "UDP receiver thread stopped.");
    }

    void publish_odom_message(const OdomData& data)
    {
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

        // 1. ヘッダー情報
        odom_msg->header.stamp = this->now();
        odom_msg->header.frame_id = ODOM_FRAME_ID;
        odom_msg->child_frame_id = BASE_FRAME_ID;

        // 2. 位置情報
        odom_msg->pose.pose.position.x = data.x;
        odom_msg->pose.pose.position.y = data.y;
        odom_msg->pose.pose.position.z = 0.0;

        // 姿勢 (Yaw角をQuaternionに変換)
        tf2::Quaternion q;
        q.setRPY(0, 0, data.yaw);
        odom_msg->pose.pose.orientation = tf2::toMsg(q);

        // 3. 速度情報
        odom_msg->twist.twist.linear.x = data.lin_vel_x;
        odom_msg->twist.twist.angular.z = data.ang_vel_z;

        // 4. パブリッシュ
        odom_publisher_->publish(std::move(odom_msg));

        // 標準出力への表示（ログ出力）も維持
        RCLCPP_INFO(this->get_logger(), 
            "--- Odom ID %u: X=%.2f, Yaw=%.2f, Vel_X=%.2f ---", 
            data.frame_id, data.x, data.yaw, data.lin_vel_x);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // ROSのメインスレッドでノードを実行 (UDP受信は別スレッド)
    rclcpp::spin(std::make_shared<OdomViewNode>());
    rclcpp::shutdown();
    return 0;
}