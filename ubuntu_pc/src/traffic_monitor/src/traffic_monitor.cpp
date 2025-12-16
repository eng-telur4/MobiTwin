#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <fstream>     // ファイル操作のために追加
#include <cinttypes> // PRIu64 マクロのために必要
#include <stdexcept>
#include <chrono>
#include <ctime>       // 時刻の整形のために追加

using livox_ros_driver2::msg::CustomMsg;
using std::placeholders::_1;
using namespace std::chrono;

// --- 日時をファイル名に含めるためのヘルパー関数 ---
std::string get_datetime_string() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
    std::tm local_time;
    if (localtime_r(&now_time, &local_time) == nullptr) {
         return "datetime_error"; 
    }

    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
    
    return std::string(buffer);
}

// --- グローバル変数: ログファイル名 ---
const std::string LOG_FILENAME_BASE = "traffic_info";

class TrafficMonitor : public rclcpp::Node
{
public:
    TrafficMonitor()
        : Node("traffic_monitor_node"), received_count_(0)
    {
        // ログファイル名生成と開く処理
        setup_log_file();

        // ROS 2トピックを購読
        // トピック名: /livox/lidar (以前のご要望通り)
        subscription_ = this->create_subscription<CustomMsg>(
            "/livox/lidar", 
            10, // QoSキューサイズ
            std::bind(&TrafficMonitor::message_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Traffic Monitor Node started. Subscribing to /livox/lidar.");
    }

    ~TrafficMonitor() override
    {
        if (output_log_file_.is_open()) {
            output_log_file_.close();
            RCLCPP_INFO(this->get_logger(), "Traffic log file closed.");
        }
    }

private:
    rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
    uint64_t received_count_; // 受信したフレームの総数
    std::ofstream output_log_file_; // ログファイルオブジェクト

    // ログファイルを開くセットアップ関数
    void setup_log_file()
    {
        std::string filename = LOG_FILENAME_BASE + "_" + get_datetime_string() + ".log";
        
        // ファイルを追記モードで開く
        output_log_file_.open(filename, std::ios::out | std::ios::app);
        if (!output_log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", filename.c_str());
            throw std::runtime_error("Could not open traffic log file.");
        }
        RCLCPP_INFO(this->get_logger(), "Traffic log opened: %s", filename.c_str());
    }

    void message_callback(const CustomMsg::SharedPtr msg)
    {
        // 1. 受信フレーム数をインクリメント
        received_count_++;
        
        // 2. Livox CustomMsgから正しいフィールドを参照して統計情報を取得
        uint32_t total_points = msg->point_num; 
        uint64_t ros_timestamp_ns = (uint64_t)msg->header.stamp.sec * 1000000000 + msg->header.stamp.nanosec;
        
        // 3. ログメッセージ文字列を整形 (RCLCPP_INFOと同じ内容)
        char log_buffer[256];
        int len = std::snprintf(log_buffer, sizeof(log_buffer), 
                                "Frame %" PRIu64 ": Points: %u | Time: %" PRIu64 " ns", 
                                received_count_, total_points, ros_timestamp_ns);

        // 4. コンソールログ出力
        // Note: RCLCPP_INFOはROSのログシステムを使うため、上記で作成したバッファを間接的に使用
        RCLCPP_INFO(this->get_logger(), "%s", log_buffer);
        
        // 5. 【修正点】ファイルにログを書き込み
        if (output_log_file_.is_open()) {
            output_log_file_ << log_buffer << "\n"; // バッファの内容をファイルに書き込み
            output_log_file_.flush(); // 即座にディスクに書き込む
        }
    }
};

int main(int argc, char *argv[])
{
    // C++17の要件があるため、CMakeLists.txtでcxx_std_17が有効であることを確認してください。
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<TrafficMonitor>());
    } catch (const std::exception& e) {
        std::cerr << "Node initialization error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}