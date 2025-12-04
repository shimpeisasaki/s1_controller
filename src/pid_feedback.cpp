#include <memory>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// ==========================================
// シンプルなPIDクラス
// ==========================================
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double limit_i)
        : kp_(kp), ki_(ki), kd_(kd), limit_i_(limit_i),
          prev_error_(0.0), integral_(0.0) {}

    // パラメータ更新用
    void set_gains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    // PID計算
    double update(double target, double current, double dt) {
        if (dt <= 0.0001) return 0.0; // ゼロ除算防止

        double error = target - current;

        // P項
        double p_term = kp_ * error;

        // I項 (アンチワインドアップ付き)
        integral_ += error * dt;
        // 積分値が大きくなりすぎないように制限
        integral_ = std::clamp(integral_, -limit_i_, limit_i_);
        double i_term = ki_ * integral_;

        // D項
        double d_term = kd_ * (error - prev_error_) / dt;
        prev_error_ = error;

        return p_term + i_term + d_term;
    }

    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double limit_i_;     // 積分の最大値
    double prev_error_;
    double integral_;
};

// ==========================================
// ROS 2 ノード
// ==========================================
class S1FeedbackController : public rclcpp::Node {
public:
    S1FeedbackController() : Node("s1_feedback_controller") {
        // --- パラメータの宣言 (デフォルト値) ---
        // X軸 (前後)
        this->declare_parameter("kp_x", 1.0);
        this->declare_parameter("ki_x", 0.1);
        this->declare_parameter("kd_x", 0.05);
        // Y軸 (横移動)
        this->declare_parameter("kp_y", 1.0);
        this->declare_parameter("ki_y", 0.1);
        this->declare_parameter("kd_y", 0.05);
        // Z軸 (回転)
        this->declare_parameter("kp_z", 0.8);
        this->declare_parameter("ki_z", 0.1);
        this->declare_parameter("kd_z", 0.01);
        
        // 積分リミット (適当な値)
        this->declare_parameter("integral_limit", 1.0);

        double i_lim = this->get_parameter("integral_limit").as_double();

        // PIDコントローラの初期化
        pid_x_ = std::make_unique<PIDController>(
            this->get_parameter("kp_x").as_double(),
            this->get_parameter("ki_x").as_double(),
            this->get_parameter("kd_x").as_double(), i_lim);

        pid_y_ = std::make_unique<PIDController>(
            this->get_parameter("kp_y").as_double(),
            this->get_parameter("ki_y").as_double(),
            this->get_parameter("kd_y").as_double(), i_lim);

        pid_z_ = std::make_unique<PIDController>(
            this->get_parameter("kp_z").as_double(),
            this->get_parameter("ki_z").as_double(),
            this->get_parameter("kd_z").as_double(), i_lim);

        // --- Pub/Subの設定 ---
        // 目標値 (cmd_vel)
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&S1FeedbackController::cmd_vel_callback, this, _1));

        // 現在値 (odom)
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&S1FeedbackController::odom_callback, this, _1));

        // 出力 (cmd_vel_out)
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);

        // タイマー (50Hz = 20ms)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&S1FeedbackController::control_loop, this));

        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "S1 Feedback Controller Started.");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_twist_ = *msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // odomのtwistは通常 base_link 座標系
        current_twist_ = msg->twist.twist;
    }

    void control_loop() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // --- パラメータのリアルタイム更新 ---
        if (this->has_parameter("kp_x")) {
              pid_x_->set_gains(
                this->get_parameter("kp_x").as_double(),
                this->get_parameter("ki_x").as_double(),
                this->get_parameter("kd_x").as_double()
             );
        }
        if (this->has_parameter("kp_y")) {
             pid_y_->set_gains(
                this->get_parameter("kp_y").as_double(),
                this->get_parameter("ki_y").as_double(),
                this->get_parameter("kd_y").as_double()
             );
        }
        if (this->has_parameter("kp_z")) {
             pid_z_->set_gains(
                this->get_parameter("kp_z").as_double(),
                this->get_parameter("ki_z").as_double(),
                this->get_parameter("kd_z").as_double()
             );
        }

        // --- ▼▼▼ ここからが追記修正部分です ▼▼▼ ---

        // PID計算
        double u_x = pid_x_->update(target_twist_.linear.x, current_twist_.linear.x, dt);
        double u_y = pid_y_->update(target_twist_.linear.y, current_twist_.linear.y, dt);
        double u_z = pid_z_->update(target_twist_.angular.z, current_twist_.angular.z, dt);

        // 最終出力の作成 (目標 + 補正)
        auto out_msg = geometry_msgs::msg::Twist();
        out_msg.linear.x = target_twist_.linear.x + u_x;
        out_msg.linear.y = target_twist_.linear.y + u_y;
        out_msg.angular.z = target_twist_.angular.z + u_z;

        // Publish
        pub_cmd_->publish(out_msg);

    } // <--- ここで関数を閉じる必要があります

    // メンバ変数
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist target_twist_;
    geometry_msgs::msg::Twist current_twist_;
    rclcpp::Time last_time_;

    std::unique_ptr<PIDController> pid_x_;
    std::unique_ptr<PIDController> pid_y_;
    std::unique_ptr<PIDController> pid_z_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<S1FeedbackController>());
    rclcpp::shutdown();
    return 0;
}
