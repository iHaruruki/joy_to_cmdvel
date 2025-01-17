#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToCmdVel : public rclcpp::Node
{
public:
  JoyToCmdVel() : Node("joy_to_cmd_vel")
  {
    // /joy トピックを購読
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

    // /cmd_vel トピックへパブリッシュするパブリッシャの作成
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "joy_to_cmd_vel_node has been started.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // geometry_msgs::msg::Twist メッセージの作成
    auto twist = geometry_msgs::msg::Twist();

    //---通常モード---//
    // 左スティックの上下（axes[1]）で前後移動、左スティックの左右（axes[0]）で横移動、右スティックの左右（axes[3]）で角速度を制御
    twist.linear.x  = 0.3 * msg->axes[1];  // 前後の移動
    twist.linear.y  = 0.3 * msg->axes[3];  // 横の移動
    twist.linear.z  = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 3.5 * msg->axes[0];  // 旋回

    //---高速モード---//
    // R1ボタン（buttons[5]）が押されている場合は速度を通常モードの1.5倍にする
    if (msg->buttons[5] == 1) {
        RCLCPP_INFO(this->get_logger(), "高速モード");
        twist.linear.x  *= 1.5;
        twist.linear.y  *= 1.5;
        twist.angular.z *= 1.5;
    }

    // /cmd_vel トピックへパブリッシュ
    cmd_vel_pub_->publish(twist);

    RCLCPP_INFO(
      this->get_logger(),
      "cmd_vel published: linear.x=%.2f, angular.z=%.2f",
      twist.linear.x, twist.angular.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyToCmdVel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
