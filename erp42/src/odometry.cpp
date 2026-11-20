#include <functional>
#include <memory>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace erp42
{
// Sample the chassis independently of the controller that owns the wheel joints.
class OdometryPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    node_ = gazebo_ros::Node::Get(sdf);
    if (sdf->HasElement("publish_tf")) {
      publish_tf_ = sdf->Get<bool>("publish_tf");
    }
    chassis_ = model->GetLink("structure");
    if (!chassis_) {
      RCLCPP_ERROR(node_->get_logger(), "Missing ERP42 chassis link 'structure'");
      return;
    }
    publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (publish_tf_) {
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    }
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&OdometryPlugin::publish, this, std::placeholders::_1));
  }

private:
  void publish(const gazebo::common::UpdateInfo & info)
  {
    if (info.simTime >= last_sample_ && info.simTime - last_sample_ < gazebo::common::Time(0, 50000000)) {
      return;
    }
    last_sample_ = info.simTime;
    const auto pose = chassis_->WorldPose();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(info.simTime);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "structure";
    odom.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
    odom.twist.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
      chassis_->RelativeLinearVel());
    odom.twist.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
      chassis_->RelativeAngularVel());
    publisher_->publish(odom);

    if (broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = odom.child_frame_id;
      transform.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
      transform.transform.rotation = odom.pose.pose.orientation;
      broadcaster_->sendTransform(transform);
    }
  }

  gazebo_ros::Node::SharedPtr node_;
  gazebo::physics::LinkPtr chassis_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  bool publish_tf_{true};
  gazebo::common::Time last_sample_;
  gazebo::event::ConnectionPtr update_;
};

GZ_REGISTER_MODEL_PLUGIN(OdometryPlugin)
}  // namespace erp42
