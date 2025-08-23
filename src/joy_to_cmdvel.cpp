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

    // デッドゾーン設定
    auto apply_deadzone = [](float value, float threshold = 0.1f) {
      return (std::abs(value) < threshold) ? 0.0f : value;
    };

    //---通常モード---//
    twist.linear.x  = 0.3f * apply_deadzone(msg->axes[1]);  // 前後の移動
    twist.linear.y  = 0.3f * apply_deadzone(msg->axes[0]);  // 横の移動
    twist.linear.z  = 0.0f;
    twist.angular.x = 0.0f;
    twist.angular.y = 0.0f;
    twist.angular.z = 0.0f;

    // angular.z の制御
    if(msg->buttons[6] == 1 && msg->buttons[7] == 0){
      twist.angular.z = 1.5f * apply_deadzone(msg->axes[2]);
    } else if(msg->buttons[7] == 1 && msg->buttons[6] == 0){
      twist.angular.z = -1.5f * apply_deadzone(msg->axes[5]);
    } else {
      twist.angular.z = 0.0f;
    }

    //---高速モード---//
    // R1ボタン（buttons[5]）が押されている場合は速度を通常モードの1.5倍にする
    // if (msg->buttons[5] == 1) {
    //     RCLCPP_INFO(this->get_logger(), "โหมดรวดเร็ว");
    //     RCLCPP_INFO(this->get_logger(), "Fast Mode");
    //     twist.linear.x  *= 1.5;
    //     twist.linear.y  *= 1.5;
    //     twist.angular.z *= 1.5;
    // }

    // /cmd_vel トピックへパブリッシュ
    cmd_vel_pub_->publish(twist);

    RCLCPP_INFO(
      this->get_logger(),
      "linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
      twist.linear.x, twist.linear.y, twist.angular.z);
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
