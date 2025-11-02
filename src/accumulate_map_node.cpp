#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "fast_limo/srv/save_map.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <deque>

using fast_limo::srv::SaveMap;

class AccumulateMapNode : public rclcpp::Node {
public:
  AccumulateMapNode() : Node("fast_limo_accumulate_map")
  {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/fast_limo/pointcloud");
    odom_topic_  = this->declare_parameter<std::string>("odom_topic", "/fast_limo/state");
    output_topic_= this->declare_parameter<std::string>("output_topic", "/fast_limo/final_map");
    world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
    leaf_size_   = this->declare_parameter<double>("voxel_leaf_size", 0.05);
    max_points_  = this->declare_parameter<int>("max_points", 5'000'000);
    max_age_sec_ = this->declare_parameter<double>("max_age_sec", 0.0);
    accept_delay_sec_ = this->declare_parameter<double>("accept_delay_sec", 0.2); // a bit more tolerant
    min_publish_period_sec_ = this->declare_parameter<double>("min_publish_period_sec", 0.5); // throttle publish-on-update
    publish_period_sec_ = this->declare_parameter<double>("publish_period_sec", 0.0); // 0 -> no timer

    // Latched (transient_local) final map publisher
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable().keep_last(1);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, latched_qos);

    // LiDAR topics often behave better with SensorDataQoS
    auto sensor_qos = rclcpp::SensorDataQoS();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, sensor_qos,
      std::bind(&AccumulateMapNode::onCloud, this, std::placeholders::_1));

    // odom is small; reliable is fine
    rclcpp::QoS odom_qos(50);
    odom_qos.reliable();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      std::bind(&AccumulateMapNode::onOdom, this, std::placeholders::_1));

    // Services unchanged...
    save_srv_ = this->create_service<SaveMap>("/fast_limo/save_map",
      std::bind(&AccumulateMapNode::onSave, this, std::placeholders::_1, std::placeholders::_2));
    send_srv_ = this->create_service<std_srvs::srv::Trigger>("/fast_limo/send_pointcloud",
      std::bind(&AccumulateMapNode::onSend, this, std::placeholders::_1, std::placeholders::_2));
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>("/fast_limo/reset_map",
      std::bind(&AccumulateMapNode::onReset, this, std::placeholders::_1, std::placeholders::_2));

    // OPTIONAL: periodic auto-publish
    if (publish_period_sec_ > 0.0) {
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(publish_period_sec_)),
        std::bind(&AccumulateMapNode::publishLatched, this));
    }

    last_pub_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Accumulating %s with odom %s -> publishing on %s",
                cloud_topic_.c_str(), odom_topic_.c_str(), output_topic_.c_str());
  }

private:
  struct TimedOdom {
    rclcpp::Time stamp;
    Eigen::Isometry3d T_w_b;
  };
  void publishLatched()
  {
    std::scoped_lock lk(mutex_);
    if (!map_ || map_->empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg);
    msg.header.stamp = now();
    msg.header.frame_id = last_frame_id_.empty() ? world_frame_ : last_frame_id_;
    map_pub_->publish(msg);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    TimedOdom t;
    t.stamp = msg->header.stamp;
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
    Eigen::Vector3d     P(p.x, p.y, p.z);
    t.T_w_b = Eigen::Isometry3d::Identity();
    t.T_w_b.linear() = Q.toRotationMatrix();
    t.T_w_b.translation() = P;

    std::scoped_lock lk(mutex_);
    odom_buf_.push_back(t);
    // keep only last few seconds
    while (odom_buf_.size() > 2000) odom_buf_.pop_front();
  }

  bool lookupPose(const rclcpp::Time& stamp, Eigen::Isometry3d& T_w_b_out)
  {
    std::scoped_lock lk(mutex_);
    if (odom_buf_.empty()) return false;

    // find closest by time
    const auto s = stamp.nanoseconds();
    double best_dt = 1e9;
    bool found = false;
    for (const auto& t : odom_buf_) {
      double dt = std::abs((t.stamp - stamp).seconds());
      if (dt < best_dt) { best_dt = dt; T_w_b_out = t.T_w_b; found = true; }
    }
    return (found && best_dt <= accept_delay_sec_);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    Eigen::Isometry3d T_w_b;
    if (!lookupPose(msg->header.stamp, T_w_b)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
        "No odom near cloud stamp; skipping frame.");
      return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr in(new pcl::PointCloud<pcl::PointXYZI>);
    try {
      pcl::fromROSMsg(*msg, *in);
    } catch (...) {
      // fallback if no intensity
      pcl::PointCloud<pcl::PointXYZ> in_xyz;
      pcl::fromROSMsg(*msg, in_xyz);
      in->reserve(in_xyz.size());
      for (auto &p : in_xyz.points) {
        pcl::PointXYZI pi; pi.x=p.x; pi.y=p.y; pi.z=p.z; pi.intensity=0.f; in->push_back(pi);
      }
    }

    // Transform each point: p_w = T_w_b * p_b
    for (auto &p : in->points) {
      Eigen::Vector3d pb(p.x, p.y, p.z);
      Eigen::Vector3d pw = T_w_b * pb;
      p.x = static_cast<float>(pw.x());
      p.y = static_cast<float>(pw.y());
      p.z = static_cast<float>(pw.z());
    }

    {
      std::scoped_lock lk(mutex_);
      if (!map_) map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      *map_ += *in;

      // Downsample if requested
      if (leaf_size_ > 0.0) {
        pcl::VoxelGrid<pcl::PointXYZI> vox;
        vox.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vox.setInputCloud(map_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr f(new pcl::PointCloud<pcl::PointXYZI>);
        vox.filter(*f);
        map_.swap(f);
      }

      if (static_cast<int>(map_->size()) > max_points_) {
        RCLCPP_WARN(get_logger(), "Map reached max_points (%d), clearing.", max_points_);
        map_->clear();
      }
      last_frame_id_ = world_frame_;
    }

    // Auto publish (throttled)
    const auto now_t = this->now();
    if ((now_t - last_pub_time_).seconds() >= min_publish_period_sec_) {
      publishLatched();
      last_pub_time_ = now_t;
    }
  }

  // --- Services ---
  void onSave(const SaveMap::Request::SharedPtr req, SaveMap::Response::SharedPtr res)
  {
    std::scoped_lock lk(mutex_);
    if (!map_ || map_->empty()) {
      res->ok = false; res->message = "No map available.";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr to_save(new pcl::PointCloud<pcl::PointXYZI>(*map_));
    if (req->crop) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
      out->reserve(to_save->size());
      for (const auto& p : to_save->points) {
        if (p.z >= req->min_z && p.z <= req->max_z) out->push_back(p);
      }
      out->width = static_cast<uint32_t>(out->size());
      out->height = 1; out->is_dense = false;
      to_save.swap(out);
    }
    int code = req->binary ? pcl::io::savePCDFileBinary(req->path, *to_save)
                           : pcl::io::savePCDFileASCII (req->path, *to_save);
    if (code == 0) { res->ok = true; res->message = "Saved: " + req->path; }
    else { res->ok = false; res->message = "PCD write error code: " + std::to_string(code); }
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void onSend(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::scoped_lock lk(mutex_);
    if (!map_ || map_->empty()) {
      res->success = false; res->message = "No map to publish."; return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg);
    msg.header.stamp = now();
    msg.header.frame_id = last_frame_id_.empty() ? world_frame_ : last_frame_id_;
    map_pub_->publish(msg);
    res->success = true; res->message = "Published on " + output_topic_;
  }

  void onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::scoped_lock lk(mutex_);
    if (map_) map_->clear();
    res->success = true; res->message = "Map cleared.";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // --- members ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<SaveMap>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr send_srv_, reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_pub_time_;
  double min_publish_period_sec_;
  double publish_period_sec_;
  std::mutex mutex_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_;
  std::deque<TimedOdom> odom_buf_;

  std::string cloud_topic_, odom_topic_, output_topic_, world_frame_, last_frame_id_;
  double leaf_size_, max_age_sec_, accept_delay_sec_;
  int max_points_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AccumulateMapNode>());
  rclcpp::shutdown();
  return 0;
}

