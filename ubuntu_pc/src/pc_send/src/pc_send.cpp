#include <iostream>                     // 標準入出力（RCLCPPのログで使用）
#include <rclcpp/rclcpp.hpp>            // ROS 2 C++クライアントライブラリの基本ヘッダー
#include <sensor_msgs/msg/point_cloud2.hpp>     // 点群メッセージのヘッダー
#include <sys/socket.h>                 // ソケットAPIの基本関数（socket, sendtoなど）
#include <arpa/inet.h>                  // インターネットアドレス操作関数（inet_pton, htonsなど）
#include <unistd.h>                     // POSIX API（close関数など）
#include <cstring>                      // メモリ操作関数（memset, memcpyなど）
#include <stdexcept>                    // 標準例外クラス（runtime_errorなど）
#include <algorithm>                    // std::min関数用
#include <fstream>                      // ファイル操作のためにfstreamヘッダーを追加
#include <chrono>                       // 時間取得用
#include <ctime>                        // 時間変換用

// --- [設定] UDP送信先の情報 ---
const char* UDP_SERVER_IP = "192.168.20.100";   // Windows PCのIPアドレス (環境に合わせて変更してください)
const int UDP_SERVER_PORT = 5000;               // 受信側のポート番号
// -----------------------------

// ヘッダー構造体のバイトサイズ
const size_t HEADER_SIZE = 20;                  // UdpHeaderのサイズ

// --- UDPチャンク設定 ---
const int CHUNK_SIZE = 4096;                    // 4 KB

// 1回のパケットの最大サイズ (ヘッダー + データチャンク)
const size_t MAX_PACKET_SIZE = HEADER_SIZE + CHUNK_SIZE;

// 受信側でデータの整合性を確認するためのヘッダー構造体
struct UdpHeader {
    uint32_t frame_id;                          // 0: この点群フレームのID
    uint32_t total_size;                        // 4: この点群フレーム全体のバイナリサイズ
    uint32_t chunk_index;                       // 8: チャンクの順番 (0, 1, 2, ...)
    uint32_t num_chunks;                        // 12: 総チャンク数
    uint32_t data_size;                         // 16: このチャンクに含まれる実際のデータサイズ (<= CHUNK_SIZE)
}; // ヘッダーサイズは20バイト

using sensor_msgs::msg::PointCloud2;            // 型名のエイリアス
using std::placeholders::_1;                    // std::bindのプレースホルダー

// ROS 2ノードを定義するクラス
class PointCloudUdpSender : public rclcpp::Node {
public:
    // コンストラクタ：ノードの初期設定
    PointCloudUdpSender() : Node("pc_send_node") {
        // 1. メンバー変数の初期化
        this->current_frame_id_ = 0;
        
        // 2. UDPソケットの設定
        this->setup_udp_socket(); 
        
        // 3. サブスクリプションの作成（QoSキューサイズ=10）
        this->subscription_ = this->create_subscription<PointCloud2>(
            "/converted_pointcloud2", 
            10, 
            std::bind(&PointCloudUdpSender::topic_callback, this, _1)
        );
        
        // 4. ノードの起動ログを出力
        RCLCPP_INFO(this->get_logger(), "Node pc_send started. Sending data to %s:%d (Target: %u frames)", 
                    UDP_SERVER_IP, UDP_SERVER_PORT, MAX_SEND_FRAMES);
    }

    // デストラクタ：ノード終了時に呼び出される
    ~PointCloudUdpSender() override {
        if (this->udp_socket_fd_ >= 0) {
            close(this->udp_socket_fd_);              // ソケットをクローズ
        }
    }

private:
    int udp_socket_fd_ = -1;                    // UDPソケットのファイルディスクリプタ
    struct sockaddr_in server_addr_;            // サーバのアドレス構造体
    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_; // サブスクリプションオブジェクト
    
    uint32_t current_frame_id_;                 // 現在送信中のフレームID
    
    // ★追加: 最大送信フレーム数
    const uint32_t MAX_SEND_FRAMES = 5000;

    uint8_t send_buffer_[MAX_PACKET_SIZE];      // チャンク送信のための一時バッファ

    // PointCloud2メッセージ受信時に呼び出されるコールバック関数
    void topic_callback(const PointCloud2::SharedPtr msg) {
        
        // ★追加: 送信上限チェック
        // current_frame_id_ は送信関数内でインクリメントされるため、
        // 0からスタートして、5000回送信が終わると5000になっています。
        if (this->current_frame_id_ >= MAX_SEND_FRAMES) {
            static bool logged_completion = false;
            if (!logged_completion) {
                RCLCPP_INFO(this->get_logger(), "TRANSMISSION STOPPED: Reached limit of %u frames.", MAX_SEND_FRAMES);
                logged_completion = true;
            }
            return; // これ以上処理せずリターン
        }

        // 初回のみダンプ (current_frame_id_ == 0 の時)
        if (!this->current_frame_id_) this->dump_data_to_file(msg);
        if (!this->current_frame_id_) this->print_and_dump_message(msg);
        
        // 1. デバッグ表示
        this->display_debug_points(msg); 
        
        // 2. UDP送信 (ここで current_frame_id_ がインクリメントされます)
        this->send_pointcloud_udp(msg->data.size(), msg->data.data(), msg->width * msg->height);
    }

    // UDPソケットのセットアップ関数
    void setup_udp_socket() {
        if ((this->udp_socket_fd_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "socket() failed");
            throw std::runtime_error("Failed to create UDP socket.");
        }

        std::memset(&this->server_addr_, 0, sizeof(this->server_addr_));
        this->server_addr_.sin_family = AF_INET;
        this->server_addr_.sin_port = htons(UDP_SERVER_PORT);

        if (inet_pton(AF_INET, UDP_SERVER_IP, &this->server_addr_.sin_addr) <= 0) {
            RCLCPP_FATAL(this->get_logger(), "inet_pton() failed for IP: %s", UDP_SERVER_IP);
            close(this->udp_socket_fd_);
            throw std::runtime_error("Invalid server IP address.");
        }
        RCLCPP_INFO(this->get_logger(), "UDP socket setup successful.");
    }

    std::string get_timestamp_string() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_time;
        if (localtime_r(&now_time, &local_time) == nullptr) {
             return ""; 
        }
        char buffer[32];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
        return std::string(buffer);
    }

    void dump_data_to_file(const PointCloud2::SharedPtr msg) {
        const std::string filename = "pointcloud_data_" + get_timestamp_string() + ".bin";
        std::ofstream outfile(filename, std::ios::out | std::ios::binary);
        if (!outfile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for dumping: %s", filename.c_str());
            return;
        }
        const char* data_ptr = reinterpret_cast<const char*>(msg->data.data());
        outfile.write(data_ptr, msg->data.size());
        outfile.close();
        RCLCPP_INFO(this->get_logger(), "Successfully dumped %zu bytes to %s", msg->data.size(), filename.c_str());
    }

    void print_and_dump_message(const PointCloud2::SharedPtr msg) {
        const std::string filename = "analyzed_points_" + get_timestamp_string() + ".txt";
        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", filename.c_str());
            return;
        }

        int x_offset = -1, y_offset = -1, z_offset = -1, i_offset = -1;
        int point_step = msg->point_step;
        size_t total_points = msg->width * msg->height;
        
        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "intensity") i_offset = field.offset;
        }

        if (x_offset == -1 || y_offset == -1 || z_offset == -1 || i_offset == -1) {
            RCLCPP_ERROR(this->get_logger(), "Required fields (x, y, z, intensity) not found in message header!");
            outfile.close();
            return;
        }

        outfile << "Index,X,Y,Z,Intensity\n"; 
        const uint8_t* data_ptr = msg->data.data();
        
        for (size_t i = 0; i < total_points; ++i) {
            size_t point_start_index = i * point_step;
            const float* x_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + x_offset);
            const float* y_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + y_offset);
            const float* z_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + z_offset);
            const float* i_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + i_offset);
            outfile << i << "," << *x_ptr << "," << *y_ptr << "," << *z_ptr << "," << *i_ptr << "\n";
        }
        outfile.close();
        RCLCPP_INFO(this->get_logger(), "Successfully analyzed and dumped %zu points to %s", total_points, filename.c_str());
    }

    void display_debug_points(const PointCloud2::SharedPtr msg) {
        int x_offset = -1, y_offset = -1, z_offset = -1, i_offset = -1;
        int point_step = msg->point_step;
        size_t total_points = msg->width * msg->height;
        const int points_to_display = 1;
        const uint8_t* data_ptr = msg->data.data();

        // ログがうるさくなるため、デバッグ表示も間引くか、必要に応じてコメントアウトしてください
        // ここでは毎回表示します

        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "intensity") i_offset = field.offset;
        }

        if (x_offset == -1 || y_offset == -1 || z_offset == -1) return;

        RCLCPP_INFO(this->get_logger(), "--- Frame %u: First point ---", this->current_frame_id_);

        for (int i = 0; i < points_to_display && i < (int)total_points; ++i) {
            size_t point_start_index = i * point_step;
            const float* x_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + x_offset);
            const float* y_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + y_offset);
            const float* z_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + z_offset);
            const float* i_ptr = reinterpret_cast<const float*>(data_ptr + point_start_index + i_offset);

            RCLCPP_INFO(this->get_logger(), "Point: x: %.3f, y: %.3f, z: %.3f, i: %.3f",
                        *x_ptr, *y_ptr, *z_ptr, *i_ptr);
        }
    }

    void send_pointcloud_udp(size_t total_data_size, const uint8_t* data_ptr, size_t total_points) {
        size_t num_chunks = (total_data_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
        size_t bytes_sent = 0;

        // フレームIDをここでインクリメント (1から始まります)
        current_frame_id_++; 

        for (uint32_t i = 0; i < num_chunks; ++i) {
            size_t offset = i * CHUNK_SIZE;
            size_t current_chunk_data_size = std::min((size_t)CHUNK_SIZE, total_data_size - offset);

            UdpHeader header;
            header.frame_id = current_frame_id_;
            header.total_size = total_data_size;
            header.chunk_index = i;
            header.num_chunks = num_chunks;
            header.data_size = current_chunk_data_size;

            std::memcpy(this->send_buffer_, &header, HEADER_SIZE); 
            std::memcpy(this->send_buffer_ + HEADER_SIZE, data_ptr + offset, current_chunk_data_size);

            size_t packet_size = HEADER_SIZE + current_chunk_data_size;
            ssize_t sent_bytes = sendto(
                this->udp_socket_fd_, 
                this->send_buffer_, 
                packet_size, 
                0, 
                (struct sockaddr *)&this->server_addr_, 
                sizeof(this->server_addr_)
            );

            if (sent_bytes != (ssize_t)packet_size) {
                RCLCPP_ERROR(this->get_logger(), "Chunk send failed!");
                break; 
            }
            bytes_sent += current_chunk_data_size;
        }

        if (bytes_sent == total_data_size) {
            // 送信成功ログ (頻度が高すぎる場合はコメントアウト推奨)
            // RCLCPP_INFO(this->get_logger(), "Sent Frame %u", current_frame_id_);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PointCloudUdpSender>());
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Node setup error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}