#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

class JoyToCmdVel_TriggersLR : public rclcpp::Node
{
public:
  JoyToCmdVel_TriggersLR() : Node("joy_to_cmd_vel_triggers_lr")
  {
    // --- パラメータ ---
    // 左スティック
    declare_parameter<int>("axis_forward", 1);     // 縦
    declare_parameter<int>("axis_strafe",  0);     // 横
    declare_parameter<bool>("invert_forward", false);   // 上=前進にするなら true
    declare_parameter<bool>("invert_strafe",  false);  // 必要なら反転

    // 最大速度
    declare_parameter<double>("max_linear_x", 0.5); // m/s
    declare_parameter<double>("max_linear_y", 0.5); // m/s
    declare_parameter<double>("max_angular",  0.5); // rad/s

    // デッドゾーン
    declare_parameter<double>("deadzone", 0.10);      // 左スティック用
    declare_parameter<double>("trigger_deadzone", 0.05); // トリガ軸用

    // L2/R2 の取得方法（ボタン or 軸）
    declare_parameter<bool>("triggers_are_axes", true); // PS系で軸になる機種は true
    declare_parameter<int>("button_L2", 6);
    declare_parameter<int>("button_R2", 7);
    declare_parameter<int>("axis_L2",   2);
    declare_parameter<int>("axis_R2",   5);

    // 取得
    axis_forward_ = get_parameter("axis_forward").as_int();
    axis_strafe_  = get_parameter("axis_strafe").as_int();
    inv_fwd_      = get_parameter("invert_forward").as_bool();
    inv_strf_     = get_parameter("invert_strafe").as_bool();

    vx_max_ = get_parameter("max_linear_x").as_double();
    vy_max_ = get_parameter("max_linear_y").as_double();
    wz_max_ = get_parameter("max_angular").as_double();

    dz_     = get_parameter("deadzone").as_double();
    tdz_    = get_parameter("trigger_deadzone").as_double();

    trig_axes_ = get_parameter("triggers_are_axes").as_bool();
    btn_L2_ = get_parameter("button_L2").as_int();
    btn_R2_ = get_parameter("button_R2").as_int();
    axis_L2_ = get_parameter("axis_L2").as_int();
    axis_R2_ = get_parameter("axis_R2").as_int();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      std::bind(&JoyToCmdVel_TriggersLR::joy_cb, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "Started joy_to_cmd_vel_triggers_lr");
  }

private:
  static double clamp(double v, double lo, double hi){
    return std::max(lo, std::min(hi, v));
  }
  double dz(double v) const { return (std::abs(v) < dz_) ? 0.0 : v; }

  // 軸値取得（範囲外は0）
  static double get_axis(const sensor_msgs::msg::Joy::SharedPtr& m, int i){
    if (i < 0 || i >= (int)m->axes.size()) return 0.0;
    return (double)m->axes[i];
  }
  // ボタン取得（範囲外は0）
  static int get_button(const sensor_msgs::msg::Joy::SharedPtr& m, int i){
    if (i < 0 || i >= (int)m->buttons.size()) return 0;
    return m->buttons[i];
  }
  // PS系のトリガ軸を 0..1 に正規化（非押下=+1, 押下=-1 → 0..1）
  double trigger01(double axis_value) const {
    double v = (1.0 - axis_value) * 0.5;  // +1→0, -1→1
    return (v < tdz_) ? 0.0 : v;          // 小さな揺れを除去
  }

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // --- 左スティックで全方向移動 ---
    double fwd  = dz(get_axis(msg, axis_forward_));
    double strf = dz(get_axis(msg, axis_strafe_));
    if (inv_fwd_)  fwd  = -fwd;
    if (inv_strf_) strf = -strf;

    double vx = clamp(vx_max_ * fwd,  -vx_max_, vx_max_);
    double vy = clamp(vy_max_ * strf, -vy_max_, vy_max_);

    // --- 旋回は L2 / R2 のみ ---
    // L2: 左旋回(＋wz), R2: 右旋回(−wz)
    double wz = 0.0;
    if (trig_axes_) {
      double l2 = trigger01(get_axis(msg, axis_L2_));
      double r2 = trigger01(get_axis(msg, axis_R2_));
      wz = wz_max_ * (l2 - r2);
    } else {
      int l2 = get_button(msg, btn_L2_) ? 1 : 0;
      int r2 = get_button(msg, btn_R2_) ? 1 : 0;
      wz = wz_max_ * (double(l2) - double(r2));
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = vx;
    cmd.linear.y  = vy;
    cmd.angular.z = clamp(wz, -wz_max_, wz_max_);
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 100,
      "vx=%.2f vy=%.2f wz=%.2f  (mode:%s)", vx, vy, cmd.angular.z,
      trig_axes_ ? "axes" : "buttons");
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  int axis_forward_, axis_strafe_;
  bool inv_fwd_, inv_strf_;
  double vx_max_, vy_max_, wz_max_;
  double dz_, tdz_;

  bool trig_axes_;
  int btn_L2_, btn_R2_, axis_L2_, axis_R2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel_TriggersLR>());
  rclcpp::shutdown();
  return 0;
}
