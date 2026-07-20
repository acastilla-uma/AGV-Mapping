#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <std_srvs/Empty.h>
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

class AccumulatorNode {
public:
  AccumulatorNode()
    : nh_("~"), tfBuffer_(), tfListener_(tfBuffer_) {
    nh_.param<bool>("enable_lidar", enable_lidar_, true);
    nh_.param<bool>("enable_camera", enable_camera_, false);

    if (!nh_.getParam("lidar_topic", lidar_topic_)) {
      nh_.param<std::string>("input_topic", lidar_topic_, "/registered_cloud");
    }
    nh_.param<std::string>("camera_topic", camera_topic_, "/camera/depth/color/points");
    nh_.param<std::string>("camera_depth_topic", camera_depth_topic_, "/camera/aligned_depth_to_color/image_raw");
    nh_.param<std::string>("camera_color_topic", camera_color_topic_, "/camera/color/image_raw");
    nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
    nh_.param<bool>("enable_camera_color", enable_camera_color_, true);
    nh_.param<bool>("use_aligned_depth_for_camera", use_aligned_depth_for_camera_, true);
    nh_.param<std::string>("target_frame", target_frame_, "map");
    nh_.param<std::string>("camera_visualization_frame", camera_visualization_frame_, "camera_depth_optical_frame");
    nh_.param<double>("voxel_size", voxel_size_, 0.05);
    nh_.param<double>("lidar_voxel_size", lidar_voxel_size_, voxel_size_);
    nh_.param<double>("camera_voxel_size", camera_voxel_size_, 0.05);
    nh_.param<double>("camera_visualization_voxel_size", camera_visualization_voxel_size_, 0.02);
    nh_.param<double>("camera_accumulate_rate", camera_accumulate_rate_, 1.0);
    nh_.param<double>("camera_visualization_rate", camera_visualization_rate_, 5.0);
    nh_.param<double>("camera_min_range", camera_min_range_, 0.20);
    nh_.param<double>("camera_max_range", camera_max_range_, 5.0);
    nh_.param<int>("camera_depth_pixel_step", camera_depth_pixel_step_, 2);
    nh_.param<std::string>("output_pcd", output_pcd_, "/tmp/accumulated_cloud.pcd");
    nh_.param<bool>("save_lidar", save_lidar_, true);
    nh_.param<bool>("save_camera", save_camera_, true);
    nh_.param<double>("camera_intensity", camera_intensity_, 0.0);
    nh_.param<double>("transform_timeout", transform_timeout_, 0.5);
    nh_.param<bool>("use_latest_tf_on_failure", use_latest_tf_on_failure_, false);
    nh_.param<std::string>("mapping_profile", mapping_profile_, "quality");
    nh_.param<std::string>("run_git_commit", run_git_commit_, "unknown");
    nh_.param<std::string>("capture_manifest", capture_manifest_, "");
    nh_.param<std::string>("camera_outlier_filter", camera_outlier_filter_, "none");
    nh_.param<int>("camera_sor_mean_k", camera_sor_mean_k_, 24);
    nh_.param<double>("camera_sor_stddev_mul", camera_sor_stddev_mul_, 1.0);
    nh_.param<double>("camera_ror_radius", camera_ror_radius_, 0.08);
    nh_.param<int>("camera_ror_min_neighbors", camera_ror_min_neighbors_, 3);
    nh_.param<int>("camera_min_points", camera_min_points_, 30);
    nh_.param<double>("camera_keyframe_min_translation", camera_keyframe_min_translation_, 0.0);
    nh_.param<double>("camera_keyframe_min_rotation_deg", camera_keyframe_min_rotation_deg_, 0.0);
    nh_.param<double>("camera_sync_tolerance", camera_sync_tolerance_, 0.03);
    nh_.param<int>("camera_sync_queue_size", camera_sync_queue_size_, 10);

    validateParametersOrThrow();

    ROS_INFO("PCD save sources: LiDAR=%s Camera=%s",
             save_lidar_ ? "true" : "false",
             save_camera_ ? "true" : "false");
    ROS_INFO("Mapping profile=%s git_commit=%s capture_manifest=%s",
             mapping_profile_.c_str(),
             run_git_commit_.c_str(),
             capture_manifest_.empty() ? "<none>" : capture_manifest_.c_str());

    if (enable_lidar_) {
      lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &AccumulatorNode::lidarCallback, this);
      ROS_INFO("LiDAR accumulation enabled: %s", lidar_topic_.c_str());
    }
    if (enable_camera_) {
      if (use_aligned_depth_for_camera_) {
        camera_depth_sub_ = nh_.subscribe(camera_depth_topic_, 2, &AccumulatorNode::cameraDepthCallback, this);
        ROS_INFO("RealSense RGB cloud from aligned depth enabled: depth=%s image=%s info=%s",
                 camera_depth_topic_.c_str(), camera_color_topic_.c_str(), camera_info_topic_.c_str());
      } else {
        camera_sub_ = nh_.subscribe(camera_topic_, 2, &AccumulatorNode::cameraCallback, this);
        ROS_INFO("RealSense pointcloud accumulation enabled: %s", camera_topic_.c_str());
      }
      if (enable_camera_color_) {
        camera_color_sub_ = nh_.subscribe(camera_color_topic_, 1, &AccumulatorNode::cameraColorCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &AccumulatorNode::cameraInfoCallback, this);
        ROS_INFO("RealSense colorization enabled: image=%s info=%s",
                 camera_color_topic_.c_str(), camera_info_topic_.c_str());
      }
      ROS_INFO("RealSense accumulation rate %.2f Hz, visualization rate %.2f Hz, accumulated voxel %.3f m, instant voxel %.3f m, range %.2f-%.2f m",
               camera_accumulate_rate_, camera_visualization_rate_,
               camera_voxel_size_, camera_visualization_voxel_size_,
               camera_min_range_, camera_max_range_);
      ROS_INFO("RealSense quality gates: min_points=%d outlier_filter=%s sor(mean_k=%d stddev=%.3f) ror(radius=%.3f min_neighbors=%d) keyframe(min_translation=%.3f min_rotation_deg=%.3f) sync_tolerance=%.3fs queue=%d",
               camera_min_points_,
               camera_outlier_filter_.c_str(),
               camera_sor_mean_k_,
               camera_sor_stddev_mul_,
               camera_ror_radius_,
               camera_ror_min_neighbors_,
               camera_keyframe_min_translation_,
               camera_keyframe_min_rotation_deg_,
               camera_sync_tolerance_,
               camera_sync_queue_size_);
    }
    if (!enable_lidar_ && !enable_camera_) {
      ROS_WARN("Both enable_lidar and enable_camera are false; no clouds will be accumulated.");
    }

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulated_points", 1, true);
    lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulated_lidar_points", 1, true);
    camera_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulated_camera_points", 1, true);
    camera_instant_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera/colored_points", 1, false);
    save_srv_ = nh_.advertiseService("save_accumulated", &AccumulatorNode::saveService, this);
  }

  ~AccumulatorNode() {
    saveToFile(output_pcd_);
  }

  void cameraColorCallback(const sensor_msgs::ImageConstPtr& image_msg) {
    std::lock_guard<std::mutex> lock(color_mutex_);
    color_image_queue_.push_back(image_msg);
    trimSyncQueue(color_image_queue_);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg) {
    std::lock_guard<std::mutex> lock(color_mutex_);
    camera_info_queue_.push_back(info_msg);
    trimSyncQueue(camera_info_queue_);
  }

  void cameraDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
    handleCameraFrame(depth_msg->header.stamp, [this, &depth_msg](pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
      return buildCameraRGBCloudFromAlignedDepth(*depth_msg, cloud);
    });
  }

  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    processCloud(cloud_msg, "LiDAR", true, false, 0.0, 0.0, 0.0,
                 accumulated_lidar_, lidar_pub_, lidar_voxel_size_);
  }

  void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    handleCameraFrame(cloud_msg->header.stamp, [this, &cloud_msg](pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
      return buildCameraRGBCloudFromPointCloud(*cloud_msg, cloud);
    });
  }

  template <typename BuildCloudFn>
  void handleCameraFrame(const ros::Time& stamp, BuildCloudFn build_cloud) {
    ++camera_frames_received_;
    bool should_accumulate = true;
    if (camera_accumulate_rate_ > 0.0 && !last_camera_accumulation_time_.isZero()) {
      const double dt = (stamp - last_camera_accumulation_time_).toSec();
      should_accumulate = dt < 0.0 || dt >= 1.0 / camera_accumulate_rate_;
    }

    bool should_visualize = true;
    if (camera_visualization_rate_ > 0.0 && !last_camera_visualization_time_.isZero()) {
      const double dt = (stamp - last_camera_visualization_time_).toSec();
      should_visualize = dt < 0.0 || dt >= 1.0 / camera_visualization_rate_;
    }

    if (!should_accumulate && !should_visualize) {
      ++camera_frames_skipped_rate_;
      logCameraCounters();
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB> source_rgb;
    if (!build_cloud(source_rgb)) {
      ++camera_frames_rejected_build_;
      ROS_WARN_THROTTLE(5.0, "RealSense camera frame did not produce valid RGB points.");
      logCameraCounters();
      return;
    }

    const uint64_t visualized_before = camera_frames_visualized_;
    const uint64_t accumulated_before = camera_frames_accumulated_quality_;
    processCameraRGBCloud(source_rgb, stamp, should_accumulate, should_visualize);
    if (should_visualize && camera_frames_visualized_ > visualized_before) {
        last_camera_visualization_time_ = stamp;
    }
    if (should_accumulate && camera_frames_accumulated_quality_ > accumulated_before) {
        last_camera_accumulation_time_ = stamp;
    }
  }

  bool processCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const std::string& source_name,
                    const bool use_input_intensity,
                    const bool prefilter_before_tf,
                    const double min_range,
                    const double max_range,
                    const double prefilter_voxel_size,
                    pcl::PointCloud<pcl::PointXYZI>& sensor_accumulated,
                    ros::Publisher& sensor_pub,
                    const double sensor_voxel_size) {
    sensor_msgs::PointCloud2 cloud_in = *cloud_msg;
    normalizeFrameId(cloud_in.header.frame_id);

    sensor_msgs::PointCloud2 cloud_transformed;
    try {
      if (prefilter_before_tf) {
        pcl::PointCloud<pcl::PointXYZI> filtered_source;
        convertToXYZI(cloud_in, use_input_intensity, min_range, max_range, filtered_source);
        if (filtered_source.empty()) {
          ROS_WARN_THROTTLE(5.0, "%s cloud has no valid points before transform.", source_name.c_str());
          return false;
        }
        if (prefilter_voxel_size > 0.0) {
          applyVoxelFilter(filtered_source, prefilter_voxel_size);
        }

        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(filtered_source, filtered_msg);
        filtered_msg.header = cloud_in.header;
        transformCloud(filtered_msg, cloud_transformed, target_frame_, source_name);
      } else {
        transformCloud(cloud_in, cloud_transformed, target_frame_, source_name);
      }
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(5.0, "%s transform failed: %s", source_name.c_str(), ex.what());
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI> pcl_in;
    pcl::fromROSMsg(cloud_transformed, pcl_in);
    removeInvalidPoints(pcl_in);
    if (pcl_in.empty()) {
      ROS_WARN_THROTTLE(5.0, "%s cloud has no valid points after conversion.", source_name.c_str());
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    appendAndFilter(sensor_accumulated, pcl_in, sensor_voxel_size);
    publishCloud(sensor_accumulated, sensor_pub, cloud_msg->header.stamp);

    appendAndFilter(accumulated_, pcl_in, voxel_size_);
    publishCloud(accumulated_, pub_, cloud_msg->header.stamp);
    return true;
  }

  bool processCameraRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>& source_rgb,
                             const ros::Time& stamp,
                             const bool accumulate,
                             const bool visualize) {
    if (source_rgb.empty()) {
      ROS_WARN_THROTTLE(5.0, "RealSense source RGB cloud is empty.");
      ++camera_frames_rejected_empty_;
      logCameraCounters();
      return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB> instant_rgb = source_rgb;
    if (camera_visualization_voxel_size_ > 0.0) {
      applyVoxelFilter(instant_rgb, camera_visualization_voxel_size_);
    }

    bool visualization_succeeded = !visualize;
    if (visualize) {
      sensor_msgs::PointCloud2 instant_msg;
      pcl::toROSMsg(instant_rgb, instant_msg);
      instant_msg.header.stamp = stamp;
      instant_msg.header.frame_id = instant_rgb.header.frame_id;

      sensor_msgs::PointCloud2 instant_transformed_msg;
      try {
        transformCloud(instant_msg, instant_transformed_msg, camera_visualization_frame_, "RealSense visualization");
        pcl::PointCloud<pcl::PointXYZRGB> instant_transformed_rgb;
        pcl::fromROSMsg(instant_transformed_msg, instant_transformed_rgb);
        removeInvalidPoints(instant_transformed_rgb);
        std::lock_guard<std::mutex> lock(mutex_);
        publishCloud(instant_transformed_rgb, camera_instant_pub_, camera_visualization_frame_, stamp);
        visualization_succeeded = true;
        ++camera_frames_visualized_;
      } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(5.0, "RealSense visualization transform failed: %s", ex.what());
      }
    }

    if (!accumulate) {
      return visualization_succeeded;
    }

    const std::string source_frame = normalizedFrameId(source_rgb.header.frame_id);
    if (!tfBuffer_.canTransform(target_frame_, source_frame, stamp, ros::Duration(0.0))) {
      ROS_WARN_THROTTLE(5.0,
                        "RealSense visualization is active, but accumulation is waiting for TF %s <- %s.",
                        target_frame_.c_str(), source_frame.c_str());
      ++camera_frames_rejected_tf_;
      logCameraCounters();
      return visualization_succeeded;
    }

    sensor_msgs::PointCloud2 source_msg;
    pcl::toROSMsg(source_rgb, source_msg);
    source_msg.header.stamp = stamp;
    source_msg.header.frame_id = source_rgb.header.frame_id;

    geometry_msgs::TransformStamped camera_transform;
    try {
      camera_transform = lookupTransformForCloud(target_frame_, source_frame, stamp, "RealSense");
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(5.0, "RealSense transform failed: %s", ex.what());
      ++camera_frames_rejected_tf_;
      logCameraCounters();
      return false;
    }

    sensor_msgs::PointCloud2 cloud_transformed;
    try {
      tf2::doTransform(source_msg, cloud_transformed, camera_transform);
      cloud_transformed.header.frame_id = target_frame_;
      cloud_transformed.header.stamp = stamp;
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "RealSense transform failed: %s", ex.what());
      ++camera_frames_rejected_tf_;
      logCameraCounters();
      return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB> diagnostic_rgb;
    pcl::fromROSMsg(cloud_transformed, diagnostic_rgb);
    removeInvalidPoints(diagnostic_rgb);
    if (diagnostic_rgb.empty()) {
      ROS_WARN_THROTTLE(5.0, "RealSense cloud has no valid RGB points after transform.");
      ++camera_frames_rejected_empty_;
      logCameraCounters();
      return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB> quality_rgb = diagnostic_rgb;
    std::string reject_reason;
    bool accepted_for_quality = true;

    if (camera_voxel_size_ > 0.0) {
      applyVoxelFilter(quality_rgb, camera_voxel_size_);
    }

    if (accepted_for_quality && camera_min_points_ > 0 &&
        quality_rgb.size() < static_cast<std::size_t>(camera_min_points_)) {
      accepted_for_quality = false;
      reject_reason = "min_points";
      ++camera_frames_rejected_min_points_;
    }

    if (accepted_for_quality && !applyCameraOutlierFilter(quality_rgb, reject_reason)) {
      accepted_for_quality = false;
      ++camera_frames_rejected_filter_;
    }

    if (accepted_for_quality &&
        !isCameraKeyframeAccepted(camera_transform, reject_reason)) {
      accepted_for_quality = false;
      ++camera_frames_rejected_keyframe_;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    ++camera_frames_base_valid_;

    if (!accepted_for_quality) {
      ROS_INFO_THROTTLE(5.0,
                        "RealSense camera frame rejected from quality map: reason=%s points=%zu",
                        reject_reason.c_str(),
                        quality_rgb.size());
      publishCloud(accumulated_camera_rgb_, camera_pub_, stamp);
      logCameraCounters();
      return visualization_succeeded;
    }

    pcl::PointCloud<pcl::PointXYZI> xyzi_in;
    convertRGBToXYZI(quality_rgb, xyzi_in);

    appendAndFilter(accumulated_camera_rgb_, quality_rgb, camera_voxel_size_);
    publishCloud(accumulated_camera_rgb_, camera_pub_, stamp);

    appendAndFilter(accumulated_camera_, xyzi_in, camera_voxel_size_);
    appendAndFilter(accumulated_, xyzi_in, voxel_size_);
    publishCloud(accumulated_, pub_, stamp);
    last_camera_keyframe_transform_ = camera_transform;
    has_last_camera_keyframe_transform_ = true;
    ++camera_frames_accumulated_quality_;
    logCameraCounters();
    return true;
  }


  bool saveService(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return saveToFile(output_pcd_);
  }

  bool saveToFile(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string snapshot_id = makeSnapshotId();
    const std::string lidar_path = appendSuffixToPath(path, "_lidar");
    const std::string camera_path = appendSuffixToPath(path, "_camera");
    const std::string fused_quality_path = appendSuffixToPath(path, "_fused_quality");
    const std::string manifest_path = replaceSuffixAndExtension(path, "_manifest", ".json");

    bool saved_any = false;
    std::vector<SavedArtifact> artifacts;

    if (save_lidar_ && !accumulated_lidar_.empty()) {
      const pcl::PointCloud<pcl::PointXYZI>& lidar_to_save = accumulated_lidar_;
      if (savePointCloudAtomically(lidar_path, lidar_to_save, "LiDAR-only")) {
        saved_any = true;
        artifacts.push_back(describeSavedArtifact("lidar", lidar_path));
      }
    } else if (!save_lidar_) {
      ROS_INFO("LiDAR-only save disabled; not saving %s", lidar_path.c_str());
    } else {
      ROS_WARN("LiDAR-only cloud is empty; not saving %s", lidar_path.c_str());
    }

    if (save_camera_ && !accumulated_camera_rgb_.empty()) {
      const pcl::PointCloud<pcl::PointXYZRGB>& camera_to_save = accumulated_camera_rgb_;
      if (savePointCloudAtomically(camera_path, camera_to_save, "RGB camera quality")) {
        saved_any = true;
        artifacts.push_back(describeSavedArtifact("camera_quality", camera_path));
      }
    } else if (!save_camera_) {
      ROS_INFO("Camera-only save disabled; not saving %s", camera_path.c_str());
    } else {
      ROS_WARN("Camera-only cloud is empty; not saving %s", camera_path.c_str());
    }

    pcl::PointCloud<pcl::PointXYZRGB> fused_rgb;
    if (save_lidar_) {
      appendXYZIAsGray(accumulated_lidar_, fused_rgb);
    }
    if (save_camera_) {
      fused_rgb += accumulated_camera_rgb_;
    }
    if (save_lidar_ && save_camera_ && !fused_rgb.empty()) {
      if (voxel_size_ > 0.0) {
        applyVoxelFilter(fused_rgb, voxel_size_);
      }
      if (savePointCloudAtomically(fused_quality_path, fused_rgb, "fused_quality LiDAR+RGB camera")) {
        saved_any = true;
        artifacts.push_back(describeSavedArtifact("fused_quality", fused_quality_path));
      }
    } else if (!save_lidar_ || !save_camera_) {
      ROS_INFO("Fused quality save disabled because LiDAR=%s Camera=%s; not saving %s",
               save_lidar_ ? "true" : "false",
               save_camera_ ? "true" : "false",
               fused_quality_path.c_str());
    } else {
      ROS_WARN("Fused quality cloud is empty; not saving %s", fused_quality_path.c_str());
    }

    if (saved_any) {
      if (!writeManifestAtomically(manifest_path, snapshot_id, artifacts)) {
        ROS_ERROR("Snapshot %s wrote PCD files but failed to publish commit-last manifest %s.",
                  snapshot_id.c_str(), manifest_path.c_str());
        return false;
      }
      ROS_INFO("Saved coherent snapshot %s with %zu artifacts and manifest %s",
               snapshot_id.c_str(), artifacts.size(), manifest_path.c_str());
    }

    return saved_any;
  }

private:
  struct SavedArtifact {
    std::string role;
    std::string path;
    std::string hash_algorithm;
    std::string hash;
    long long size_bytes;
  };

  void validateParametersOrThrow() {
    const std::string filter = lower(camera_outlier_filter_);
    camera_outlier_filter_ = filter;

    if (voxel_size_ < 0.0 || lidar_voxel_size_ < 0.0 ||
        camera_voxel_size_ < 0.0 || camera_visualization_voxel_size_ < 0.0) {
      throw std::runtime_error("Voxel sizes must be non-negative.");
    }
    if (camera_min_range_ > 0.0 && camera_max_range_ > 0.0 &&
        camera_min_range_ >= camera_max_range_) {
      throw std::runtime_error("camera_min_range must be smaller than camera_max_range.");
    }
    if (camera_depth_pixel_step_ <= 0) {
      throw std::runtime_error("camera_depth_pixel_step must be positive.");
    }
    if (transform_timeout_ < 0.0 || camera_sync_tolerance_ < 0.0) {
      throw std::runtime_error("transform_timeout and camera_sync_tolerance must be non-negative.");
    }
    if (camera_sync_queue_size_ < 1) {
      throw std::runtime_error("camera_sync_queue_size must be at least 1.");
    }
    if (camera_min_points_ < 0) {
      throw std::runtime_error("camera_min_points must be non-negative.");
    }
    if (camera_keyframe_min_translation_ < 0.0 || camera_keyframe_min_rotation_deg_ < 0.0) {
      throw std::runtime_error("camera keyframe thresholds must be non-negative.");
    }
    if (filter != "none" && filter != "sor" && filter != "ror") {
      throw std::runtime_error("camera_outlier_filter must be one of: none, sor, ror.");
    }
    if (filter == "sor" && (camera_sor_mean_k_ < 2 || camera_sor_stddev_mul_ <= 0.0)) {
      throw std::runtime_error("SOR requires camera_sor_mean_k >= 2 and camera_sor_stddev_mul > 0.");
    }
    if (filter == "ror" && (camera_ror_radius_ <= 0.0 || camera_ror_min_neighbors_ < 1)) {
      throw std::runtime_error("ROR requires camera_ror_radius > 0 and camera_ror_min_neighbors >= 1.");
    }
    if (use_latest_tf_on_failure_) {
      ROS_WARN("use_latest_tf_on_failure=true is enabled. This is for diagnostics only; quality mapping should keep it false.");
    }
  }

  std::string lower(std::string value) const {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
  }

  geometry_msgs::TransformStamped lookupTransformForCloud(const std::string& target_frame,
                                                          const std::string& source_frame,
                                                          const ros::Time& stamp,
                                                          const std::string& source_name) const {
    const ros::Duration timeout(transform_timeout_);
    try {
      return tfBuffer_.lookupTransform(target_frame, source_frame, stamp, timeout);
    } catch (tf2::TransformException& ex) {
      if (!use_latest_tf_on_failure_) {
        throw;
      }
      try {
        geometry_msgs::TransformStamped latest =
            tfBuffer_.lookupTransform(target_frame, source_frame, ros::Time(0), timeout);
        latest.header.stamp = stamp;
        ROS_WARN_THROTTLE(5.0,
                          "%s transform at sensor time failed (%s); used latest available TF instead.",
                          source_name.c_str(), ex.what());
        return latest;
      } catch (tf2::TransformException&) {
        throw ex;
      }
    }
  }

  bool applyCameraOutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                std::string& reject_reason) const {
    if (camera_outlier_filter_ == "none" || cloud.empty()) {
      return true;
    }

    const std::size_t before = cloud.size();
    pcl::PointCloud<pcl::PointXYZRGB> filtered;

    if (camera_outlier_filter_ == "sor") {
      if (cloud.size() <= static_cast<std::size_t>(camera_sor_mean_k_)) {
        reject_reason = "sor_insufficient_points";
        return false;
      }
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(cloud.makeShared());
      sor.setMeanK(camera_sor_mean_k_);
      sor.setStddevMulThresh(camera_sor_stddev_mul_);
      sor.filter(filtered);
    } else if (camera_outlier_filter_ == "ror") {
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
      ror.setInputCloud(cloud.makeShared());
      ror.setRadiusSearch(camera_ror_radius_);
      ror.setMinNeighborsInRadius(camera_ror_min_neighbors_);
      ror.filter(filtered);
    }

    cloud.swap(filtered);
    if (cloud.empty()) {
      reject_reason = camera_outlier_filter_ + "_removed_all_points";
      return false;
    }
    if (camera_min_points_ > 0 && cloud.size() < static_cast<std::size_t>(camera_min_points_)) {
      reject_reason = camera_outlier_filter_ + "_below_min_points";
      return false;
    }

    ROS_INFO_THROTTLE(5.0,
                      "RealSense %s filter kept %zu/%zu points before accumulation.",
                      camera_outlier_filter_.c_str(), cloud.size(), before);
    return true;
  }

  bool isCameraKeyframeAccepted(const geometry_msgs::TransformStamped& current,
                                std::string& reject_reason) const {
    if (!has_last_camera_keyframe_transform_) {
      return true;
    }
    if (camera_keyframe_min_translation_ <= 0.0 && camera_keyframe_min_rotation_deg_ <= 0.0) {
      return true;
    }

    const geometry_msgs::Vector3& a = last_camera_keyframe_transform_.transform.translation;
    const geometry_msgs::Vector3& b = current.transform.translation;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    const double translation = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double rotation_deg = quaternionAngularDistanceDeg(last_camera_keyframe_transform_.transform.rotation,
                                                             current.transform.rotation);

    const bool moved_enough =
        camera_keyframe_min_translation_ <= 0.0 || translation >= camera_keyframe_min_translation_;
    const bool rotated_enough =
        camera_keyframe_min_rotation_deg_ <= 0.0 || rotation_deg >= camera_keyframe_min_rotation_deg_;

    if (!moved_enough && !rotated_enough) {
      std::ostringstream reason;
      reason << "keyframe_delta_small translation=" << translation
             << " rotation_deg=" << rotation_deg;
      reject_reason = reason.str();
      return false;
    }
    return true;
  }

  double quaternionAngularDistanceDeg(const geometry_msgs::Quaternion& q1,
                                      const geometry_msgs::Quaternion& q2) const {
    double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    dot = std::fabs(dot);
    dot = std::min(1.0, std::max(-1.0, dot));
    const double pi = 3.14159265358979323846;
    return 2.0 * std::acos(dot) * 180.0 / pi;
  }

  template <typename MsgPtrT>
  void trimSyncQueue(std::deque<MsgPtrT>& queue) const {
    while (queue.size() > static_cast<std::size_t>(camera_sync_queue_size_)) {
      queue.pop_front();
    }
  }

  template <typename MsgPtrT>
  MsgPtrT findClosestStampedMessage(const std::deque<MsgPtrT>& queue,
                                    const ros::Time& stamp,
                                    double& best_delta) const {
    MsgPtrT best;
    best_delta = std::numeric_limits<double>::infinity();
    for (const auto& msg : queue) {
      const double delta = std::fabs((msg->header.stamp - stamp).toSec());
      if (delta < best_delta) {
        best_delta = delta;
        best = msg;
      }
    }
    if (best && best_delta <= camera_sync_tolerance_) {
      return best;
    }
    return MsgPtrT();
  }

  bool findSyncedColorAndInfo(const ros::Time& stamp,
                              sensor_msgs::ImageConstPtr& image_msg,
                              sensor_msgs::CameraInfoConstPtr& info_msg,
                              std::string& reject_reason) {
    std::lock_guard<std::mutex> lock(color_mutex_);

    double info_delta = 0.0;
    info_msg = findClosestStampedMessage(camera_info_queue_, stamp, info_delta);
    if (!info_msg) {
      std::ostringstream reason;
      reason << "camera_info_sync_missing tolerance_s=" << camera_sync_tolerance_;
      reject_reason = reason.str();
      return false;
    }

    image_msg.reset();
    if (enable_camera_color_) {
      double image_delta = 0.0;
      image_msg = findClosestStampedMessage(color_image_queue_, stamp, image_delta);
      if (!image_msg) {
        std::ostringstream reason;
        reason << "color_sync_missing tolerance_s=" << camera_sync_tolerance_;
        reject_reason = reason.str();
        return false;
      }
      ROS_INFO_THROTTLE(10.0,
                        "Aligned RGB-D sync deltas: color=%.4fs camera_info=%.4fs tolerance=%.4fs",
                        image_delta, info_delta, camera_sync_tolerance_);
    }

    return true;
  }

  void logCameraCounters() {
    ROS_INFO_THROTTLE(10.0,
                      "RealSense counters: received=%llu base_valid=%llu visualized=%llu quality_accumulated=%llu skipped_rate=%llu reject_build=%llu reject_tf=%llu reject_empty=%llu reject_min_points=%llu reject_filter=%llu reject_keyframe=%llu reject_sync=%llu",
                      static_cast<unsigned long long>(camera_frames_received_),
                      static_cast<unsigned long long>(camera_frames_base_valid_),
                      static_cast<unsigned long long>(camera_frames_visualized_),
                      static_cast<unsigned long long>(camera_frames_accumulated_quality_),
                      static_cast<unsigned long long>(camera_frames_skipped_rate_),
                      static_cast<unsigned long long>(camera_frames_rejected_build_),
                      static_cast<unsigned long long>(camera_frames_rejected_tf_),
                      static_cast<unsigned long long>(camera_frames_rejected_empty_),
                      static_cast<unsigned long long>(camera_frames_rejected_min_points_),
                      static_cast<unsigned long long>(camera_frames_rejected_filter_),
                      static_cast<unsigned long long>(camera_frames_rejected_keyframe_),
                      static_cast<unsigned long long>(camera_frames_rejected_sync_));
  }

  bool transformCloud(const sensor_msgs::PointCloud2& input,
                      sensor_msgs::PointCloud2& output,
                      const std::string& target_frame,
                      const std::string& source_name) const {
    const ros::Duration timeout(transform_timeout_);
    try {
      tfBuffer_.transform(input, output, target_frame, timeout);
      return true;
    } catch (tf2::TransformException& ex) {
      if (!use_latest_tf_on_failure_) {
        throw;
      }

      sensor_msgs::PointCloud2 latest_input = input;
      latest_input.header.stamp = ros::Time(0);
      try {
        tfBuffer_.transform(latest_input, output, target_frame, timeout);
        output.header.stamp = input.header.stamp;
        ROS_WARN_THROTTLE(5.0,
                          "%s transform at sensor time failed (%s); used latest available TF instead.",
                          source_name.c_str(), ex.what());
        return true;
      } catch (tf2::TransformException&) {
        throw ex;
      }
    }
  }

  template <typename PointT>
  bool savePointCloudAtomically(const std::string& path,
                                const pcl::PointCloud<PointT>& cloud,
                                const std::string& label) const {
    const std::string tmp_path = path + ".tmp";
    const int result = pcl::io::savePCDFileBinary(tmp_path, cloud);
    if (result != 0) {
      ROS_ERROR("Failed to write %s cloud to temporary file %s (PCL error %d).",
                label.c_str(), tmp_path.c_str(), result);
      return false;
    }

    std::ifstream tmp_stream(tmp_path.c_str(), std::ios::binary | std::ios::ate);
    if (!tmp_stream.good() || tmp_stream.tellg() <= 0) {
      ROS_ERROR("Temporary %s cloud file %s is empty or unreadable; keeping previous output.",
                label.c_str(), tmp_path.c_str());
      std::remove(tmp_path.c_str());
      return false;
    }
    tmp_stream.close();

    if (std::rename(tmp_path.c_str(), path.c_str()) != 0) {
      ROS_ERROR("Failed to atomically move %s cloud from %s to %s: %s",
                label.c_str(), tmp_path.c_str(), path.c_str(), std::strerror(errno));
      std::remove(tmp_path.c_str());
      return false;
    }

    ROS_INFO("Saved %s cloud to %s", label.c_str(), path.c_str());
    return true;
  }

  std::string appendSuffixToPath(const std::string& path, const std::string& suffix) const {
    const std::string::size_type slash_pos = path.find_last_of('/');
    const std::string::size_type dot_pos = path.find_last_of('.');
    if (dot_pos != std::string::npos && (slash_pos == std::string::npos || dot_pos > slash_pos)) {
      return path.substr(0, dot_pos) + suffix + path.substr(dot_pos);
    }
    return path + suffix + ".pcd";
  }

  std::string replaceSuffixAndExtension(const std::string& path,
                                        const std::string& suffix,
                                        const std::string& extension) const {
    const std::string::size_type slash_pos = path.find_last_of('/');
    const std::string::size_type dot_pos = path.find_last_of('.');
    if (dot_pos != std::string::npos && (slash_pos == std::string::npos || dot_pos > slash_pos)) {
      return path.substr(0, dot_pos) + suffix + extension;
    }
    return path + suffix + extension;
  }

  std::string makeSnapshotId() {
    const ros::Time now = ros::Time::now();
    std::ostringstream stream;
    stream << now.sec << "_" << std::setw(9) << std::setfill('0') << now.nsec
           << "_" << save_sequence_++;
    return stream.str();
  }

  SavedArtifact describeSavedArtifact(const std::string& role, const std::string& path) const {
    SavedArtifact artifact;
    artifact.role = role;
    artifact.path = path;
    artifact.hash_algorithm = "fnv1a64";
    artifact.hash = fileHashFnv1a64(path);
    artifact.size_bytes = fileSizeBytes(path);
    return artifact;
  }

  long long fileSizeBytes(const std::string& path) const {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
      return -1;
    }
    return static_cast<long long>(info.st_size);
  }

  std::string fileHashFnv1a64(const std::string& path) const {
    std::ifstream stream(path.c_str(), std::ios::binary);
    if (!stream.good()) {
      return "";
    }

    uint64_t hash = 1469598103934665603ULL;
    char buffer[4096];
    while (stream.good()) {
      stream.read(buffer, sizeof(buffer));
      const std::streamsize count = stream.gcount();
      for (std::streamsize i = 0; i < count; ++i) {
        hash ^= static_cast<unsigned char>(buffer[i]);
        hash *= 1099511628211ULL;
      }
    }

    std::ostringstream out;
    out << std::hex << std::setw(16) << std::setfill('0') << hash;
    return out.str();
  }

  std::string jsonEscape(const std::string& value) const {
    std::ostringstream out;
    for (const char c : value) {
      switch (c) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
          if (static_cast<unsigned char>(c) < 0x20) {
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(static_cast<unsigned char>(c));
          } else {
            out << c;
          }
      }
    }
    return out.str();
  }

  bool writeManifestAtomically(const std::string& path,
                               const std::string& snapshot_id,
                               const std::vector<SavedArtifact>& artifacts) const {
    const std::string tmp_path = path + ".tmp";
    std::ofstream out(tmp_path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.good()) {
      ROS_ERROR("Failed to open manifest temporary file %s.", tmp_path.c_str());
      return false;
    }

    out << "{\n";
    out << "  \"schema\": \"scout_mapping_snapshot_v1\",\n";
    out << "  \"snapshot_id\": \"" << jsonEscape(snapshot_id) << "\",\n";
    out << "  \"mapping_profile\": \"" << jsonEscape(mapping_profile_) << "\",\n";
    out << "  \"run_git_commit\": \"" << jsonEscape(run_git_commit_) << "\",\n";
    out << "  \"capture_manifest\": \"" << jsonEscape(capture_manifest_) << "\",\n";
    out << "  \"target_frame\": \"" << jsonEscape(target_frame_) << "\",\n";
    out << "  \"products\": {\n";
    out << "    \"fused_quality\": \"LiDAR plus camera frames accepted by quality gates\"\n";
    out << "  },\n";
    out << "  \"parameters\": {\n";
    out << "    \"voxel_size\": " << voxel_size_ << ",\n";
    out << "    \"lidar_voxel_size\": " << lidar_voxel_size_ << ",\n";
    out << "    \"camera_voxel_size\": " << camera_voxel_size_ << ",\n";
    out << "    \"camera_min_range\": " << camera_min_range_ << ",\n";
    out << "    \"camera_max_range\": " << camera_max_range_ << ",\n";
    out << "    \"camera_outlier_filter\": \"" << jsonEscape(camera_outlier_filter_) << "\",\n";
    out << "    \"camera_sor_mean_k\": " << camera_sor_mean_k_ << ",\n";
    out << "    \"camera_sor_stddev_mul\": " << camera_sor_stddev_mul_ << ",\n";
    out << "    \"camera_ror_radius\": " << camera_ror_radius_ << ",\n";
    out << "    \"camera_ror_min_neighbors\": " << camera_ror_min_neighbors_ << ",\n";
    out << "    \"camera_min_points\": " << camera_min_points_ << ",\n";
    out << "    \"camera_keyframe_min_translation\": " << camera_keyframe_min_translation_ << ",\n";
    out << "    \"camera_keyframe_min_rotation_deg\": " << camera_keyframe_min_rotation_deg_ << ",\n";
    out << "    \"camera_sync_tolerance\": " << camera_sync_tolerance_ << ",\n";
    out << "    \"use_latest_tf_on_failure\": " << (use_latest_tf_on_failure_ ? "true" : "false") << "\n";
    out << "  },\n";
    out << "  \"counters\": {\n";
    out << "    \"camera_received\": " << camera_frames_received_ << ",\n";
    out << "    \"camera_base_valid\": " << camera_frames_base_valid_ << ",\n";
    out << "    \"camera_quality_accumulated\": " << camera_frames_accumulated_quality_ << ",\n";
    out << "    \"camera_rejected_tf\": " << camera_frames_rejected_tf_ << ",\n";
    out << "    \"camera_rejected_sync\": " << camera_frames_rejected_sync_ << ",\n";
    out << "    \"camera_rejected_min_points\": " << camera_frames_rejected_min_points_ << ",\n";
    out << "    \"camera_rejected_filter\": " << camera_frames_rejected_filter_ << ",\n";
    out << "    \"camera_rejected_keyframe\": " << camera_frames_rejected_keyframe_ << "\n";
    out << "  },\n";
    out << "  \"artifacts\": [\n";
    for (std::size_t i = 0; i < artifacts.size(); ++i) {
      const SavedArtifact& artifact = artifacts[i];
      out << "    {\"role\": \"" << jsonEscape(artifact.role)
          << "\", \"path\": \"" << jsonEscape(artifact.path)
          << "\", \"hash_algorithm\": \"" << jsonEscape(artifact.hash_algorithm)
          << "\", \"hash\": \"" << jsonEscape(artifact.hash)
          << "\", \"size_bytes\": " << artifact.size_bytes << "}";
      if (i + 1 < artifacts.size()) {
        out << ",";
      }
      out << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    out.close();

    if (!out.good()) {
      ROS_ERROR("Failed while writing manifest temporary file %s.", tmp_path.c_str());
      std::remove(tmp_path.c_str());
      return false;
    }

    if (std::rename(tmp_path.c_str(), path.c_str()) != 0) {
      ROS_ERROR("Failed to atomically publish manifest %s: %s",
                path.c_str(), std::strerror(errno));
      std::remove(tmp_path.c_str());
      return false;
    }

    return true;
  }

  void appendXYZIAsGray(const pcl::PointCloud<pcl::PointXYZI>& input,
                        pcl::PointCloud<pcl::PointXYZRGB>& output) const {
    if (input.empty()) {
      return;
    }

    float min_i = input.points.front().intensity;
    float max_i = input.points.front().intensity;
    for (const auto& point : input.points) {
      if (std::isfinite(point.intensity)) {
        min_i = std::min(min_i, point.intensity);
        max_i = std::max(max_i, point.intensity);
      }
    }
    const float range = std::max(1.0f, max_i - min_i);

    output.reserve(output.size() + input.size());
    for (const auto& point : input.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const float normalized = std::max(0.0f, std::min(1.0f, (point.intensity - min_i) / range));
      const uint8_t gray = static_cast<uint8_t>(40.0f + normalized * 180.0f);
      pcl::PointXYZRGB rgb_point;
      rgb_point.x = point.x;
      rgb_point.y = point.y;
      rgb_point.z = point.z;
      rgb_point.r = gray;
      rgb_point.g = gray;
      rgb_point.b = gray;
      output.push_back(rgb_point);
    }
    output.width = output.size();
    output.height = 1;
    output.is_dense = true;
  }

  void normalizeFrameId(std::string& frame_id) {
    while (!frame_id.empty() && frame_id[0] == '/') {
      frame_id.erase(0, 1);
    }
  }

  std::string normalizedFrameId(std::string frame_id) const {
    while (!frame_id.empty() && frame_id[0] == '/') {
      frame_id.erase(0, 1);
    }
    return frame_id;
  }

  bool hasField(const sensor_msgs::PointCloud2& cloud, const std::string& field_name) const {
    for (const auto& field : cloud.fields) {
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }

  void convertToXYZI(const sensor_msgs::PointCloud2& cloud,
                     const bool use_input_intensity,
                     const double min_range,
                     const double max_range,
                     pcl::PointCloud<pcl::PointXYZI>& output) const {
    const bool copy_intensity = use_input_intensity && hasField(cloud, "intensity");

    output.clear();
    output.header.frame_id = cloud.header.frame_id;
    output.reserve(static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_i(cloud, copy_intensity ? "intensity" : "x");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }

      const double range = std::sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y) + (*iter_z) * (*iter_z));
      if ((min_range > 0.0 && range < min_range) || (max_range > 0.0 && range > max_range)) {
        continue;
      }

      pcl::PointXYZI point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      point.intensity = copy_intensity ? *iter_i : static_cast<float>(camera_intensity_);
      output.push_back(point);
    }

    output.width = output.size();
    output.height = 1;
    output.is_dense = true;
  }

  bool buildCameraRGBCloudFromAlignedDepth(const sensor_msgs::Image& depth_msg,
                                      pcl::PointCloud<pcl::PointXYZRGB>& output) {
    sensor_msgs::ImageConstPtr image_msg;
    sensor_msgs::CameraInfoConstPtr info_msg;
    std::string sync_reject_reason;
    if (!findSyncedColorAndInfo(depth_msg.header.stamp, image_msg, info_msg, sync_reject_reason)) {
      ++camera_frames_rejected_sync_;
      ROS_WARN_THROTTLE(5.0, "Aligned depth RGB-D sync rejected frame: %s", sync_reject_reason.c_str());
      return false;
    }

    const bool use_color = enable_camera_color_ && image_msg &&
                           image_msg->width > 0 && image_msg->height > 0 &&
                           image_msg->width == depth_msg.width &&
                           image_msg->height == depth_msg.height;
    if (enable_camera_color_ && !use_color) {
      ++camera_frames_rejected_sync_;
      ROS_WARN_THROTTLE(5.0, "Aligned depth RGB image exists but dimensions do not match depth; rejecting frame instead of using latest/white fallback.");
      return false;
    }

    if (depth_msg.width == 0 || depth_msg.height == 0) {
      return false;
    }

    const double fx = info_msg->K[0];
    const double fy = info_msg->K[4];
    const double cx = info_msg->K[2];
    const double cy = info_msg->K[5];
    if (fx == 0.0 || fy == 0.0) {
      ROS_WARN_THROTTLE(5.0, "Camera intrinsics are invalid; fx/fy are zero.");
      return false;
    }

    const std::string frame_id = normalizedFrameId(!info_msg->header.frame_id.empty()
                                                    ? info_msg->header.frame_id
                                                    : depth_msg.header.frame_id);
    output.clear();
    output.header.frame_id = frame_id;
    output.reserve(static_cast<std::size_t>(depth_msg.width) * static_cast<std::size_t>(depth_msg.height));

    const int step_px = camera_depth_pixel_step_ > 0 ? camera_depth_pixel_step_ : 1;
    for (int v = 0; v < static_cast<int>(depth_msg.height); v += step_px) {
      for (int u = 0; u < static_cast<int>(depth_msg.width); u += step_px) {
        double z = 0.0;
        if (!readDepthMeters(depth_msg, u, v, z)) {
          continue;
        }
        if ((camera_min_range_ > 0.0 && z < camera_min_range_) ||
            (camera_max_range_ > 0.0 && z > camera_max_range_)) {
          continue;
        }

        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
        if (use_color && !getImageRGB(*image_msg, u, v, r, g, b)) {
          continue;
        }

        pcl::PointXYZRGB point;
        point.z = static_cast<float>(z);
        point.x = static_cast<float>((u - cx) * z / fx);
        point.y = static_cast<float>((v - cy) * z / fy);
        point.r = r;
        point.g = g;
        point.b = b;
        output.push_back(point);
      }
    }

    output.width = output.size();
    output.height = 1;
    output.is_dense = true;
    ROS_INFO_THROTTLE(5.0, "Built RGB camera cloud from aligned depth with %zu points before voxel filtering.", output.size());
    return !output.empty();
  }

  bool readDepthMeters(const sensor_msgs::Image& depth_msg, const int u, const int v, double& depth_m) const {
    if (u < 0 || v < 0 || u >= static_cast<int>(depth_msg.width) || v >= static_cast<int>(depth_msg.height)) {
      return false;
    }

    const std::size_t offset = static_cast<std::size_t>(v) * depth_msg.step + static_cast<std::size_t>(u) * bytesPerDepthPixel(depth_msg.encoding);
    if (offset >= depth_msg.data.size()) {
      return false;
    }

    if (depth_msg.encoding == "16UC1" || depth_msg.encoding == "mono16") {
      if (offset + sizeof(uint16_t) > depth_msg.data.size()) {
        return false;
      }
      uint16_t raw = 0;
      raw = static_cast<uint16_t>(depth_msg.data[offset]) |
            static_cast<uint16_t>(depth_msg.data[offset + 1]) << 8;
      if (raw == 0) {
        return false;
      }
      depth_m = static_cast<double>(raw) * 0.001;
    } else if (depth_msg.encoding == "32FC1") {
      if (offset + sizeof(float) > depth_msg.data.size()) {
        return false;
      }
      float raw = 0.0f;
      std::memcpy(&raw, &depth_msg.data[offset], sizeof(float));
      if (!std::isfinite(raw) || raw <= 0.0f) {
        return false;
      }
      depth_m = raw;
    } else {
      ROS_WARN_THROTTLE(10.0, "Unsupported depth encoding '%s'.", depth_msg.encoding.c_str());
      return false;
    }

    return std::isfinite(depth_m) && depth_m > 0.0;
  }

  std::size_t bytesPerDepthPixel(const std::string& encoding) const {
    if (encoding == "16UC1" || encoding == "mono16") {
      return sizeof(uint16_t);
    }
    if (encoding == "32FC1") {
      return sizeof(float);
    }
    return 0;
  }

  bool buildCameraRGBCloudFromPointCloud(const sensor_msgs::PointCloud2& cloud_in_msg,
                                         pcl::PointCloud<pcl::PointXYZRGB>& output) {
    sensor_msgs::PointCloud2 cloud_in = cloud_in_msg;
    normalizeFrameId(cloud_in.header.frame_id);
    if (hasField(cloud_in, "rgb") || hasField(cloud_in, "rgba")) {
      convertToXYZRGB(cloud_in, camera_min_range_, camera_max_range_, output);
      return !output.empty();
    }

    sensor_msgs::ImageConstPtr image_msg;
    sensor_msgs::CameraInfoConstPtr info_msg;

    std::string sync_reject_reason;
    if (!enable_camera_color_ ||
        !findSyncedColorAndInfo(cloud_in.header.stamp, image_msg, info_msg, sync_reject_reason)) {
      if (enable_camera_color_) {
        ROS_WARN_THROTTLE(5.0,
                          "RealSense cloud has no rgb field and synchronized color is unavailable (%s); using white fallback.",
                          sync_reject_reason.c_str());
      }
      ROS_WARN_THROTTLE(5.0, "RealSense cloud has no rgb field and color image/camera_info are not ready; using white fallback.");
      convertToXYZRGB(cloud_in, camera_min_range_, camera_max_range_, output);
      return !output.empty();
    }

    sensor_msgs::PointCloud2 cloud_for_projection = cloud_in;
    const std::string color_frame = normalizedFrameId(!info_msg->header.frame_id.empty()
                                                       ? info_msg->header.frame_id
                                                       : image_msg->header.frame_id);
    if (!color_frame.empty() && cloud_for_projection.header.frame_id != color_frame) {
      try {
        transformCloud(cloud_in, cloud_for_projection, color_frame, "RealSense color projection");
        normalizeFrameId(cloud_for_projection.header.frame_id);
      } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(5.0, "RealSense color projection transform failed: %s", ex.what());
        return false;
      }
    }

    convertToXYZRGBFromImage(cloud_for_projection, *image_msg, *info_msg,
                             camera_min_range_, camera_max_range_, output);
    return !output.empty();
  }

  void convertToXYZRGB(const sensor_msgs::PointCloud2& cloud,
                       const double min_range,
                       const double max_range,
                       pcl::PointCloud<pcl::PointXYZRGB>& output) const {
    if (!hasField(cloud, "rgb") && !hasField(cloud, "rgba")) {
      ROS_WARN_THROTTLE(10.0, "RealSense cloud has no rgb/rgba field; camera accumulated cloud will use white fallback color.");
    }

    pcl::PointCloud<pcl::PointXYZRGB> input;
    pcl::fromROSMsg(cloud, input);

    output.clear();
    output.header.frame_id = cloud.header.frame_id;
    output.reserve(input.size());

    for (const auto& point : input.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if ((min_range > 0.0 && range < min_range) || (max_range > 0.0 && range > max_range)) {
        continue;
      }

      pcl::PointXYZRGB out = point;
      if (!hasField(cloud, "rgb") && !hasField(cloud, "rgba")) {
        out.r = 255;
        out.g = 255;
        out.b = 255;
      }
      output.push_back(out);
    }

    output.width = output.size();
    output.height = 1;
    output.is_dense = true;
  }

  void convertRGBToXYZI(const pcl::PointCloud<pcl::PointXYZRGB>& input,
                        pcl::PointCloud<pcl::PointXYZI>& output) const {
    output.clear();
    output.header = input.header;
    output.reserve(input.size());

    for (const auto& rgb_point : input.points) {
      pcl::PointXYZI point;
      point.x = rgb_point.x;
      point.y = rgb_point.y;
      point.z = rgb_point.z;
      point.intensity = static_cast<float>(camera_intensity_);
      output.push_back(point);
    }

    output.width = output.size();
    output.height = 1;
    output.is_dense = true;
  }

  bool getImageRGB(const sensor_msgs::Image& image,
                   const int u,
                   const int v,
                   uint8_t& r,
                   uint8_t& g,
                   uint8_t& b) const {
    if (u < 0 || v < 0 || u >= static_cast<int>(image.width) || v >= static_cast<int>(image.height)) {
      return false;
    }

    int channels = 0;
    bool bgr = false;
    if (image.encoding == "rgb8") {
      channels = 3;
    } else if (image.encoding == "bgr8") {
      channels = 3;
      bgr = true;
    } else if (image.encoding == "rgba8") {
      channels = 4;
    } else if (image.encoding == "bgra8") {
      channels = 4;
      bgr = true;
    } else {
      ROS_WARN_THROTTLE(10.0, "Unsupported RealSense color encoding '%s'; using white fallback.", image.encoding.c_str());
      return false;
    }

    const std::size_t offset = static_cast<std::size_t>(v) * image.step + static_cast<std::size_t>(u) * channels;
    if (offset + channels > image.data.size()) {
      return false;
    }

    if (bgr) {
      b = image.data[offset + 0];
      g = image.data[offset + 1];
      r = image.data[offset + 2];
    } else {
      r = image.data[offset + 0];
      g = image.data[offset + 1];
      b = image.data[offset + 2];
    }
    return true;
  }

  void convertToXYZRGBFromImage(const sensor_msgs::PointCloud2& cloud,
                                const sensor_msgs::Image& image,
                                const sensor_msgs::CameraInfo& info,
                                const double min_range,
                                const double max_range,
                                pcl::PointCloud<pcl::PointXYZRGB>& output) const {
    const double fx = info.K[0];
    const double fy = info.K[4];
    const double cx = info.K[2];
    const double cy = info.K[5];

    output.clear();
    output.header.frame_id = cloud.header.frame_id;
    output.reserve(static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z) || *iter_z <= 0.0f) {
        continue;
      }

      const double range = std::sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y) + (*iter_z) * (*iter_z));
      if ((min_range > 0.0 && range < min_range) || (max_range > 0.0 && range > max_range)) {
        continue;
      }

      const int u = static_cast<int>(fx * (*iter_x) / (*iter_z) + cx + 0.5);
      const int v = static_cast<int>(fy * (*iter_y) / (*iter_z) + cy + 0.5);
      uint8_t r = 255;
      uint8_t g = 255;
      uint8_t b = 255;
      if (!getImageRGB(image, u, v, r, g, b)) {
        continue;
      }

      pcl::PointXYZRGB point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      point.r = r;
      point.g = g;
      point.b = b;
      output.push_back(point);
    }

    output.width = output.size();
    output.height = 1;
    output.is_dense = true;
  }

  void removeInvalidPoints(pcl::PointCloud<pcl::PointXYZI>& cloud) const {
    pcl::PointCloud<pcl::PointXYZI> filtered;
    filtered.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        filtered.push_back(point);
      }
    }
    filtered.width = filtered.size();
    filtered.height = 1;
    filtered.is_dense = true;
    cloud.swap(filtered);
  }

  void removeInvalidPoints(pcl::PointCloud<pcl::PointXYZRGB>& cloud) const {
    pcl::PointCloud<pcl::PointXYZRGB> filtered;
    filtered.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        filtered.push_back(point);
      }
    }
    filtered.width = filtered.size();
    filtered.height = 1;
    filtered.is_dense = true;
    cloud.swap(filtered);
  }

  void applyVoxelFilter(pcl::PointCloud<pcl::PointXYZI>& cloud, const double leaf_size) const {
    if (leaf_size <= 0.0 || cloud.empty()) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(tmp);
    cloud.swap(tmp);
  }

  void applyVoxelFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud, const double leaf_size) const {
    if (leaf_size <= 0.0 || cloud.empty()) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> tmp;
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(tmp);
    cloud.swap(tmp);
  }

  void appendAndFilter(pcl::PointCloud<pcl::PointXYZI>& target,
                       const pcl::PointCloud<pcl::PointXYZI>& input,
                       const double leaf_size) const {
    if (target.empty()) {
      target = input;
    } else {
      target += input;
    }
    applyVoxelFilter(target, leaf_size);
  }

  void appendAndFilter(pcl::PointCloud<pcl::PointXYZRGB>& target,
                       const pcl::PointCloud<pcl::PointXYZRGB>& input,
                       const double leaf_size) const {
    if (target.empty()) {
      target = input;
    } else {
      target += input;
    }
    applyVoxelFilter(target, leaf_size);
  }

  void publishCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                    ros::Publisher& publisher,
                    const ros::Time& stamp = ros::Time::now()) const {
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(cloud, out_msg);
    out_msg.header.frame_id = target_frame_;
    out_msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    publisher.publish(out_msg);
  }

  void publishCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                    ros::Publisher& publisher,
                    const ros::Time& stamp = ros::Time::now()) const {
    publishCloud(cloud, publisher, target_frame_, stamp);
  }

  void publishCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                    ros::Publisher& publisher,
                    const std::string& frame_id,
                    const ros::Time& stamp = ros::Time::now()) const {
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(cloud, out_msg);
    out_msg.header.frame_id = frame_id;
    out_msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    publisher.publish(out_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber camera_depth_sub_;
  ros::Subscriber camera_color_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher pub_;
  ros::Publisher lidar_pub_;
  ros::Publisher camera_pub_;
  ros::Publisher camera_instant_pub_;
  ros::ServiceServer save_srv_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  pcl::PointCloud<pcl::PointXYZI> accumulated_;
  pcl::PointCloud<pcl::PointXYZI> accumulated_lidar_;
  pcl::PointCloud<pcl::PointXYZI> accumulated_camera_;
  pcl::PointCloud<pcl::PointXYZRGB> accumulated_camera_rgb_;
  std::mutex mutex_;
  std::mutex color_mutex_;
  std::deque<sensor_msgs::ImageConstPtr> color_image_queue_;
  std::deque<sensor_msgs::CameraInfoConstPtr> camera_info_queue_;
  bool enable_lidar_;
  bool enable_camera_;
  bool enable_camera_color_;
  bool use_aligned_depth_for_camera_;
  bool save_lidar_;
  bool save_camera_;
  std::string lidar_topic_;
  std::string camera_topic_;
  std::string camera_depth_topic_;
  std::string camera_color_topic_;
  std::string camera_info_topic_;
  std::string target_frame_;
  std::string camera_visualization_frame_;
  std::string mapping_profile_;
  std::string run_git_commit_;
  std::string capture_manifest_;
  std::string camera_outlier_filter_;
  double voxel_size_;
  double lidar_voxel_size_;
  double camera_voxel_size_;
  double camera_visualization_voxel_size_;
  double camera_accumulate_rate_;
  double camera_visualization_rate_;
  double camera_min_range_;
  double camera_max_range_;
  double camera_intensity_;
  double transform_timeout_;
  double camera_sor_stddev_mul_;
  double camera_ror_radius_;
  double camera_keyframe_min_translation_;
  double camera_keyframe_min_rotation_deg_;
  double camera_sync_tolerance_;
  bool use_latest_tf_on_failure_;
  int camera_depth_pixel_step_;
  int camera_sor_mean_k_;
  int camera_ror_min_neighbors_;
  int camera_min_points_;
  int camera_sync_queue_size_;
  std::string output_pcd_;
  ros::Time last_camera_accumulation_time_;
  ros::Time last_camera_visualization_time_;
  geometry_msgs::TransformStamped last_camera_keyframe_transform_;
  bool has_last_camera_keyframe_transform_ = false;
  unsigned int save_sequence_ = 0;
  uint64_t camera_frames_received_ = 0;
  uint64_t camera_frames_base_valid_ = 0;
  uint64_t camera_frames_visualized_ = 0;
  uint64_t camera_frames_accumulated_quality_ = 0;
  uint64_t camera_frames_skipped_rate_ = 0;
  uint64_t camera_frames_rejected_build_ = 0;
  uint64_t camera_frames_rejected_tf_ = 0;
  uint64_t camera_frames_rejected_empty_ = 0;
  uint64_t camera_frames_rejected_min_points_ = 0;
  uint64_t camera_frames_rejected_filter_ = 0;
  uint64_t camera_frames_rejected_keyframe_ = 0;
  uint64_t camera_frames_rejected_sync_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_pointcloud_accumulator");
  try {
    AccumulatorNode node;
    ros::spin();
    return 0;
  } catch (const std::exception& ex) {
    ROS_FATAL("Failed to start scout_pointcloud_accumulator: %s", ex.what());
    return 1;
  }
}
