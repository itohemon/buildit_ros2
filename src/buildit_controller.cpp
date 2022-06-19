#include <memory>
#include <bits/stdc++.h>
#include <chrono>
#include <functional>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "buildit_ros2/com_buildit.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class builditControl : public rclcpp::Node
{

public:
  builditControl() : Node("buildit_control")
  {
    /*
     * パラメータファイルから設定取得
     */
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->get_parameter("port", port_name_);

    this->declare_parameter("wheel_radius", 0.0762);
    this->get_parameter("wheel_radius", wheel_radius_);

    this->declare_parameter("wheel_separation", 0.330);
    this->get_parameter("wheel_separation", wheel_separation_);

    this->declare_parameter("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame_);

    this->declare_parameter("odom_child_frame", "base_footprint");
    this->get_parameter("odom_child_frame", odom_child_frame_);

    this->declare_parameter("dxl_id_l", 1);
    this->get_parameter("dxl_id_l", id_L_);

    this->declare_parameter("dxl_id_r", 2);
    this->get_parameter("dxl_id_r", id_R_);

    this->declare_parameter("joint_state_l", "wheel_left");
    this->get_parameter("joint_state_l", joint_state_l_);

    this->declare_parameter("joint_state_r", "wheel_right");
    this->get_parameter("joint_state_r", joint_state_r_);

    g_x_ = 0.0;
    g_y_ = 0.0;
    g_th_ = 0.0;

    g_last_time_ = this->now();

    initializeComBuildIt(&fd, port_name_);

    transFreeMode(id_L_);
    transFreeMode(id_R_);

    transReadyMode(id_L_);
    transReadyMode(id_R_);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&builditControl::topic_callback, this, _1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_frame_.c_str(), 10);
    jointstate_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    timer_ = this->create_wall_timer(
        10ms, std::bind(&builditControl::odometryPublish_callback, this));

    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  /*
   * デストラクタ
   * 終了時はトルクを抜く
   */
  ~builditControl()
  {
    transFreeMode(id_L_);
    transFreeMode(id_R_);
    transHoldMode(id_L_);
    transHoldMode(id_R_);
  }

private:
  int transFreeMode(uint8_t id)
  {
    if (sendFreeCmd(fd, id) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to trans to free mode", id);
      return -1;
    }
    recvDataBuidIt(fd, buf);
  }

  int transReadyMode(uint8_t id)
  {
    if (sendReadyCmd(fd, id) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to trans to ready mode", id);
      return -1;
    }
    recvDataBuidIt(fd, buf);
  }

  int transHoldMode(uint8_t id)
  {
    if (sendHoldCmd(fd, id) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to trans to hold mode", id);
      return -1;
    }
    recvDataBuidIt(fd, buf);
  }

  /*
   * cmd_velを受けたときのコールバック関数
   * 並進・回転速度をもらって左右のモータを回す
   */
  int topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double wR, wL;
    double v = msg->linear.x;  // m/s
    double w = msg->angular.z; // rad/s
    double rpmR, rpmL;
    int16_t valR, valL;

    wR = v / wheel_radius_ + wheel_separation_ * w / 2.0 / wheel_radius_;
    wL = v / wheel_radius_ - wheel_separation_ * w / 2.0 / wheel_radius_;
    rpmR = 30.0 * wR / M_PI;
    rpmL = 30.0 * wL / M_PI;
    valR = rpmR * 100; // モータの仕様
    valL = rpmL * 100; // モータの仕様

    setGoalVelocity(id_L_, valL);
    setGoalVelocity(id_R_, valR);

    return 0;
  }

  /*
   * モータに目標速度を与える
   */
  int setGoalVelocity(uint8_t id, int16_t value)
  {
    if (sendSetRefVelocityCmd(fd, id, value) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to send goal velocity", id);
      return -1;
    }

    recvDataBuidIt(fd, buf);

    return 0;
  }

  /*
   * オドメトリ・JointState・TFをパブリッシュするコールバック関数
   */
  int odometryPublish_callback()
  {
    auto odom_msg = nav_msgs::msg::Odometry();
    auto state_msg = sensor_msgs::msg::JointState();

    int16_t preVel_L, preVel_R;
    int32_t prePos_L, prePos_R;
    double val_L, val_R;
    double rad_L, rad_R;

    rclcpp::Time current_time = this->now();

    // 現在速度取得
    getPresentVelocity(id_L_, &preVel_L);
    getPresentVelocity(id_R_, &preVel_R);

    val_L = (int)preVel_L * 0.01 * 2 * M_PI / 60;
    val_R = (int)preVel_R * 0.01 * 2 * M_PI / 60;

    // RCLCPP_INFO(this->get_logger(), "Present Velocity (%f, %f)", val_L, val_R);

    // 現在位置取得
    getPresentPosition(id_L_, &prePos_L);
    getPresentPosition(id_R_, &prePos_R);

    rad_L = prePos_L * 2 * M_PI / 4096;
    rad_R = prePos_R * 2 * M_PI / 4096;

    double vl = wheel_radius_ * val_L; // [m/s]
    double vr = wheel_radius_ * val_R; // [m/s]

    double vx = (vr + vl) / 2.0; // [m/s]
    double vy = 0.0;
    double vth = (vr - vl) / wheel_separation_; // [rad/s]

    double dt = current_time.seconds() - g_last_time_.seconds();

    double delta_x = (vx * cos(g_th_) - vy * sin(g_th_)) * dt;
    double delta_y = (vx * sin(g_th_) + vy * cos(g_th_)) * dt;
    double delta_th = vth * dt;

    g_x_ += delta_x;
    g_y_ += delta_y;
    g_th_ += delta_th;

    // Odomを作ってパブリッシュする
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_.c_str();

    odom_msg.pose.pose.position.x = g_x_;
    odom_msg.pose.pose.position.y = g_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g_th_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.child_frame_id = odom_child_frame_.c_str();
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vth;

    odom_publisher_->publish(odom_msg);

    // JointStatesを作ってパブリッシュする
    state_msg.header.stamp = current_time;
    state_msg.name.resize(2);
    state_msg.name[0] = joint_state_l_.c_str();
    state_msg.name[1] = joint_state_r_.c_str();
    state_msg.position.resize(2);
    state_msg.position[0] = rad_L;
    state_msg.position[1] = rad_R;

    jointstate_publisher_->publish(state_msg);

    // odomフレーム(TF)をブロードキャストする
    geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

    odom_tf.header.frame_id = odom_msg.header.frame_id;
    odom_tf.child_frame_id = odom_msg.child_frame_id;
    odom_tf.header.stamp = odom_msg.header.stamp;

    odom_tf_broadcaster_->sendTransform(odom_tf);

    g_last_time_ = current_time;

    return 0;
  }

  /*
   * モータから回転速度を取得する
   */
  int getPresentVelocity(uint8_t id, int16_t *velocity)
  {
    if (sendGetRefVelocityCmd(fd, id) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to get ref velocity", id);
      return -1;
    }

    recvDataBuidIt(fd, buf);

    *velocity = buf[4] << 8 | buf[3];

    return 0;
  }

  /*
   * モータから位置を取得する
   */
  int getPresentPosition(uint8_t id, int32_t *position)
  {
    if (sendGetRefPositionCmd(fd, id) != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to get ref position", id);
      return -1;
    }

    recvDataBuidIt(fd, buf);

    *position = buf[6] << 24 | buf[5] << 16 | buf[4] << 8 | buf[3];

    return 0;
  }

  std::string port_name_;

  double wheel_radius_;
  double wheel_separation_;

  std::string odom_frame_;
  std::string odom_child_frame_;

  uint8_t id_L_;
  uint8_t id_R_;

  std::string joint_state_l_;
  std::string joint_state_r_;

  double g_x_;
  double g_y_;
  double g_th_;

  rclcpp::Time g_last_time_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

  int fd;
  uint8_t buf[256];
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<builditControl>());
  rclcpp::shutdown();

  return 0;
}
