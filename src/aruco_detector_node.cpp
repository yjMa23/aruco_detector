#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

namespace
{

geometry_msgs::msg::Quaternion quaternionFromRotationMatrix(const cv::Mat & rotation)
{
  const double m00 = rotation.at<double>(0, 0);
  const double m01 = rotation.at<double>(0, 1);
  const double m02 = rotation.at<double>(0, 2);
  const double m10 = rotation.at<double>(1, 0);
  const double m11 = rotation.at<double>(1, 1);
  const double m12 = rotation.at<double>(1, 2);
  const double m20 = rotation.at<double>(2, 0);
  const double m21 = rotation.at<double>(2, 1);
  const double m22 = rotation.at<double>(2, 2);

  geometry_msgs::msg::Quaternion q;
  const double trace = m00 + m11 + m22;

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (m21 - m12) / s;
    q.y = (m02 - m20) / s;
    q.z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q.w = (m21 - m12) / s;
    q.x = 0.25 * s;
    q.y = (m01 + m10) / s;
    q.z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q.w = (m02 - m20) / s;
    q.x = (m01 + m10) / s;
    q.y = 0.25 * s;
    q.z = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q.w = (m10 - m01) / s;
    q.x = (m02 + m20) / s;
    q.y = (m12 + m21) / s;
    q.z = 0.25 * s;
  }

  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 0.0) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  } else {
    q.w = 1.0;
  }

  return q;
}

cv::Ptr<cv::aruco::Dictionary> makeDictionary(
  const std::string & name,
  const rclcpp::Logger & logger)
{
  static const std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionaries = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };

  const auto it = dictionaries.find(name);
  if (it == dictionaries.end()) {
    RCLCPP_WARN(
      logger,
      "Unknown ArUco dictionary '%s'; falling back to DICT_4X4_50",
      name.c_str());
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }

  return cv::aruco::getPredefinedDictionary(it->second);
}

}  // namespace

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info");
    marker_length_ = declare_parameter<double>("marker_length", 0.5);
    dictionary_name_ = declare_parameter<std::string>("dictionary", "DICT_4X4_50");
    target_id_ = declare_parameter<int>("target_id", 0);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);

    if (marker_length_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "marker_length must be positive; falling back to 0.5 m");
      marker_length_ = 0.5;
    }

    if (sync_queue_size_ < 1) {
      RCLCPP_WARN(
        get_logger(),
        "sync_queue_size must be at least 1; falling back to 10");
      sync_queue_size_ = 10;
    }

    dictionary_ = makeDictionary(dictionary_name_, get_logger());
    detector_params_ = cv::aruco::DetectorParameters::create();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/pose", 10);
    visible_pub_ = create_publisher<std_msgs::msg::Bool>("/aruco/visible", 10);
    debug_image_pub_ =
      create_publisher<sensor_msgs::msg::Image>("/aruco/debug_image", rclcpp::SensorDataQoS());

    image_sub_.subscribe(this, image_topic_, rmw_qos_profile_sensor_data);
    camera_info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(static_cast<uint32_t>(sync_queue_size_)),
      image_sub_,
      camera_info_sub_);
    sync_->registerCallback(
      std::bind(
        &ArucoDetectorNode::handleImage,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Detecting ArUco target id %d on %s with %s, marker_length=%.3f m",
      target_id_,
      image_topic_.c_str(),
      dictionary_name_.c_str(),
      marker_length_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void handleImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
  {
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to convert image to bgr8: %s",
        ex.what());
      publishVisible(false);
      return;
    }

    cv::Mat debug_image = cv_image->image;
    cv::Mat gray_image;
    cv::cvtColor(debug_image, gray_image, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray_image, dictionary_, corners, ids, detector_params_);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
    }

    const auto target_it = std::find(ids.begin(), ids.end(), target_id_);
    const bool target_visible = target_it != ids.end();
    const bool valid_camera_info = hasValidCameraInfo(*camera_info_msg);

    if (!target_visible || !valid_camera_info) {
      if (!valid_camera_info) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "CameraInfo has invalid intrinsics; skipping pose estimation");
      }

      publishVisible(false);
      publishDebugImage(debug_image, *image_msg);
      return;
    }

    const auto target_index = static_cast<size_t>(std::distance(ids.begin(), target_it));
    const std::vector<std::vector<cv::Point2f>> target_corners = {corners[target_index]};
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    const cv::Mat camera_matrix = cameraMatrixFromInfo(*camera_info_msg);
    const cv::Mat dist_coeffs = distortionFromInfo(*camera_info_msg);

    cv::aruco::estimatePoseSingleMarkers(
      target_corners,
      marker_length_,
      camera_matrix,
      dist_coeffs,
      rvecs,
      tvecs);

    if (rvecs.empty() || tvecs.empty()) {
      publishVisible(false);
      publishDebugImage(debug_image, *image_msg);
      return;
    }

    cv::aruco::drawAxis(
      debug_image,
      camera_matrix,
      dist_coeffs,
      rvecs.front(),
      tvecs.front(),
      marker_length_ * 0.5);

    publishPose(rvecs.front(), tvecs.front(), *image_msg);
    publishVisible(true);
    publishDebugImage(debug_image, *image_msg);
  }

  bool hasValidCameraInfo(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    return camera_info.k[0] > 0.0 &&
           camera_info.k[4] > 0.0 &&
           camera_info.k[8] != 0.0;
  }

  cv::Mat cameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    return (cv::Mat_<double>(3, 3) <<
           camera_info.k[0], camera_info.k[1], camera_info.k[2],
           camera_info.k[3], camera_info.k[4], camera_info.k[5],
           camera_info.k[6], camera_info.k[7], camera_info.k[8]);
  }

  cv::Mat distortionFromInfo(const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    if (camera_info.d.empty()) {
      return cv::Mat::zeros(1, 5, CV_64F);
    }

    cv::Mat dist_coeffs(camera_info.d, true);
    return dist_coeffs.reshape(1, 1);
  }

  void publishPose(
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec,
    const sensor_msgs::msg::Image & image_msg)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = image_msg.header;
    pose_msg.pose.position.x = tvec[0];
    pose_msg.pose.position.y = tvec[1];
    pose_msg.pose.position.z = tvec[2];
    pose_msg.pose.orientation = quaternionFromRotationMatrix(rotation_matrix);

    pose_pub_->publish(pose_msg);
  }

  void publishVisible(bool visible)
  {
    std_msgs::msg::Bool visible_msg;
    visible_msg.data = visible;
    visible_pub_->publish(visible_msg);
  }

  void publishDebugImage(
    const cv::Mat & debug_image,
    const sensor_msgs::msg::Image & source_msg)
  {
    cv_bridge::CvImage debug_msg;
    debug_msg.header = source_msg.header;
    debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
    debug_msg.image = debug_image;
    debug_image_pub_->publish(*debug_msg.toImageMsg());
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string dictionary_name_;
  double marker_length_;
  int target_id_;
  int sync_queue_size_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
