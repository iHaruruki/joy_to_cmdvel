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
    // --- 入力（左スティック、トリガ） ---
    declare_parameter<int>("axis_forward", 1);       // 左スティック 縦
    declare_parameter<int>("axis_strafe",  0);       // 左スティック 横
    declare_parameter<bool>("invert_forward", false);
    declare_parameter<bool>("invert_strafe",  false);

    // --- 最大速度（到達上限） ---
    declare_parameter<double>("max_linear_x", 0.5);  // m/s
    declare_parameter<double>("max_linear_y", 0.5);  // m/s
    declare_parameter<double>("max_angular",  0.7);  // rad/s

    // --- デッドゾーン ---
    declare_parameter<double>("deadzone", 0.10);          // スティック用
    declare_parameter<double>("trigger_deadzone", 0.05);  // トリガ軸用

    // --- L2/R2 の取得方法（ボタン or 軸） ---
    declare_parameter<bool>("triggers_are_axes", true);
    declare_parameter<int>("button_L2", 6);
    declare_parameter<int>("button_R2", 7);
    declare_parameter<int>("axis_L2",   2);
    declare_parameter<int>("axis_R2",   5);

    // --- ここがポイント：加減速リミット（安全） ---
    // 直線・平行移動は m/s^2、旋回は rad/s^2
    declare_parameter<double>("limit_lin_accel",  0.05); // 加速上限
    declare_parameter<double>("limit_lin_decel",  0.3); // 減速上限
    declare_parameter<double>("limit_ang_accel",  0.6);
    declare_parameter<double>("limit_ang_decel",  0.8);

    // 取得
    axis_forward_ = get_parameter("axis_forward").as_int();
    axis_strafe_  = get_parameter("axis_strafe").as_int();
    inv_fwd_      = get_parameter("invert_forward").as_bool();
    inv_strf_     = get_parameter("invert_strafe").as_bool();

    vx_max_ = get_parameter("max_linear_x").as_double();
    vy_max_ = get_parameter("max_linear_y").as_double();
    wz_max_ = get_parameter("max_angular").as_double();

    dz_  = get_parameter("deadzone").as_double();
    tdz_ = get_parameter("trigger_deadzone").as_double();

    trig_axes_ = get_parameter("triggers_are_axes").as_bool();
    btn_L2_ = get_parameter("button_L2").as_int();
    btn_R2_ = get_parameter("button_R2").as_int();
    axis_L2_ = get_parameter("axis_L2").as_int();
    axis_R2_ = get_parameter("axis_R2").as_int();

    lim_ax_ = get_parameter("limit_lin_accel").as_double();
    lim_dx_ = get_parameter("limit_lin_decel").as_double();
    lim_ay_ = get_parameter("limit_lin_accel").as_double();
    lim_dy_ = get_parameter("limit_lin_decel").as_double();
    lim_aw_ = get_parameter("limit_ang_accel").as_double();
    lim_dw_ = get_parameter("limit_ang_decel").as_double();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      std::bind(&JoyToCmdVel_TriggersLR::joy_cb, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    last_time_ = now();
    RCLCPP_INFO(get_logger(), "Started joy_to_cmd_vel_triggers_lr (with accel/decel limits)");
  }

private:
  static double clamp(double v, double lo, double hi){
    return std::max(lo, std::min(hi, v));
  }
  double dz(double v) const { return (std::abs(v) < dz_) ? 0.0 : v; }

  static double get_axis(const sensor_msgs::msg::Joy::SharedPtr& m, int i){
    if (i < 0 || i >= (int)m->axes.size()) return 0.0;
    return (double)m->axes[i];
  }
  static int get_button(const sensor_msgs::msg::Joy::SharedPtr& m, int i){
    if (i < 0 || i >= (int)m->buttons.size()) return 0;
    return m->buttons[i];
  }
  // PS系トリガ（軸）を 0..1 に正規化（+1→0, -1→1）
  double trigger01(double axis_value) const {
    double v = (1.0 - axis_value) * 0.5;
    return (v < tdz_) ? 0.0 : v;
  }

  // 目標 target を前回 prev から、dt秒で「最大加速/減速」に収めて更新
  static double slew_sep(double target, double prev,
                         double limit_accel, double limit_decel, double dt)
  {
    double delta = target - prev;
    double max_up   =  std::max(0.0, limit_accel) * dt;
    double max_down =  std::max(0.0, limit_decel) * dt;
    if (delta > 0.0)      delta = std::min(delta,  max_up);
    else if (delta < 0.0) delta = std::max(delta, -max_down);
    return prev + delta;
  }

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // 経過時間 dt を計算
    rclcpp::Time t = now();
    double dt = std::max(1e-3, (t - last_time_).seconds());
    last_time_ = t;

    // --- 左スティック：全方向移動（目標値） ---
    double fwd  = dz(get_axis(msg, axis_forward_));
    double strf = dz(get_axis(msg, axis_strafe_));
    if (inv_fwd_)  fwd  = -fwd;
    if (inv_strf_) strf = -strf;

    double vx_target = clamp(vx_max_ * fwd,  -vx_max_, vx_max_);
    double vy_target = clamp(vy_max_ * strf, -vy_max_, vy_max_);

    // --- 旋回は L2 / R2 ---
    // L2：左（+wz）、R2：右（-wz）
    double wz_target = 0.0;
    if (trig_axes_) {
      double l2 = trigger01(get_axis(msg, axis_L2_));
      double r2 = trigger01(get_axis(msg, axis_R2_));
      wz_target = clamp(wz_max_ * (l2 - r2), -wz_max_, wz_max_);
    } else {
      int l2 = get_button(msg, btn_L2_) ? 1 : 0;
      int r2 = get_button(msg, btn_R2_) ? 1 : 0;
      wz_target = wz_max_ * (double(l2) - double(r2));
    }

    // --- スルーレート制限（ここで“急発進・急停止”を抑制） ---
    vx_prev_ = slew_sep(vx_target, vx_prev_, lim_ax_, lim_dx_, dt);
    vy_prev_ = slew_sep(vy_target, vy_prev_, lim_ay_, lim_dy_, dt);
    wz_prev_ = slew_sep(wz_target, wz_prev_, lim_aw_, lim_dw_, dt);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = vx_prev_;
    cmd.linear.y  = vy_prev_;
    cmd.angular.z = wz_prev_;
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 200,
      "vx=%.2f vy=%.2f wz=%.2f  (dt=%.3f)", cmd.linear.x, cmd.linear.y, cmd.angular.z, dt);
  }

  // ---- メンバ ----
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  int axis_forward_, axis_strafe_;
  bool inv_fwd_, inv_strf_;
  double vx_max_, vy_max_, wz_max_;
  double dz_, tdz_;
  bool trig_axes_;
  int btn_L2_, btn_R2_, axis_L2_, axis_R2_;

  // 加減速上限
  double lim_ax_, lim_dx_; // x 方向（前後）
  double lim_ay_, lim_dy_; // y 方向（左右）
  double lim_aw_, lim_dw_; // yaw

  // ステート
  rclcpp::Time last_time_;
  double vx_prev_{0.0}, vy_prev_{0.0}, wz_prev_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel_TriggersLR>());
  rclcpp::shutdown();
  return 0;
}
