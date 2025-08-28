#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>

class JoyToCmdVel : public rclcpp::Node
{
public:
  JoyToCmdVel() : Node("joy_to_cmd_vel")
  {
    // パラメータ宣言
    declare_parameter<double>("max_linear", 0.3);   // m/s
    declare_parameter<double>("max_angular", 0.7);  // rad/s
    declare_parameter<double>("deadzone", 0.1);
    declare_parameter<double>("trigger_deadzone", 0.05);

    // パラメータ取得
    max_linear_ = get_parameter("max_linear").as_double();
    max_angular_ = get_parameter("max_angular").as_double();
    deadzone_ = get_parameter("deadzone").as_double();
    trigger_deadzone_ = get_parameter("trigger_deadzone").as_double();

    // サブスクリプションとパブリッシャー作成
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyToCmdVel::joy_cb, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "Started simple joy_to_cmd_vel node with trigger control");
  }

private:
  // デッドゾーン処理
  double apply_deadzone(double value) const {
    return (std::abs(value) < deadzone_) ? 0.0 : value;
  }

  // 軸の値を安全に取得
  double get_axis(const sensor_msgs::msg::Joy::SharedPtr& msg, int axis) const {
    if (axis < 0 || axis >= (int)msg->axes.size()) return 0.0;
    return msg->axes[axis];
  }

  // PS系トリガー（軸）を 0..1 に正規化（+1→0, -1→1）
  double normalize_trigger(double axis_value) const {
    double v = (1.0 - axis_value) * 0.5;
    return (v < trigger_deadzone_) ? 0.0 : v;
  }

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // 左スティック：前後（軸1）と左右移動（軸0）
    double linear_x = apply_deadzone(get_axis(msg, 1));   // 前後移動
    double linear_y = apply_deadzone(get_axis(msg, 0));   // 左右移動

    // 速度制限を適用
    linear_x = std::clamp(linear_x * max_linear_, -max_linear_, max_linear_);
    linear_y = std::clamp(-linear_y * max_linear_, -max_linear_, max_linear_); // 左右反転

    // トリガー：L2（軸2）で左回転（＋）/ R2（軸5）で右回転（−）
    double l2 = normalize_trigger(get_axis(msg, 2));
    double r2 = normalize_trigger(get_axis(msg, 5));
    double angular = std::clamp((l2 - r2) * max_angular_, -max_angular_, max_angular_);

    // Twistメッセージを作成・送信
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = linear_y;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);

    // ログ出力（1秒に5回まで）
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 200,
      "linear_x: %.2f, linear_y: %.2f, angular: %.2f", 
      cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  // メンバ変数
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  
  double max_linear_;
  double max_angular_;
  double deadzone_;
  double trigger_deadzone_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel>());
  rclcpp::shutdown();
  return 0;
}
