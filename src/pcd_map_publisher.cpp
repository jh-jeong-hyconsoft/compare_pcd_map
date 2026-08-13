#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcd_compare/transformation_estimation_xyz_yaw.hpp"

class PcdMapPublisher : public rclcpp::Node
{
public:
  PcdMapPublisher()
  : Node("pcd_map_publisher")
  {
    declare_parameter("ref_directory", "data/ref");
    declare_parameter("eval_directory", "data/eval");
    declare_parameter("frame_id", "map");
    declare_parameter("alignment.enabled", true);
    declare_parameter("alignment.initial_guess", "identity");
    declare_parameter("alignment.voxel_leaf_size", 0.5);
    declare_parameter("alignment.max_correspondence_distance", 3.0);
    declare_parameter("alignment.max_iterations", 100);
    declare_parameter("alignment.transformation_epsilon", 1e-8);
    declare_parameter("alignment.fitness_epsilon", 1e-6);
    declare_parameter("alignment.min_inlier_ratio", 0.5);
    declare_parameter("alignment.max_inlier_ratio_drop", 0.02);
    declare_parameter("alignment.min_relative_mse_improvement", 0.01);

    // These are consumed by compare.launch.py when it creates dynamic RViz displays.
    declare_parameter("visualization.reference_color", "255; 255; 255");
    declare_parameter<std::vector<std::string>>(
      "visualization.eval_colors",
      {"255; 64; 64", "64; 255; 64", "64; 160; 255", "255; 64; 255"});
    declare_parameter("visualization.point_size", 2);
    declare_parameter("visualization.grid_cell_size", 5.0);

    initialize();
  }

private:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;
  using Publisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

  struct Score
  {
    double mean_squared_error{std::numeric_limits<double>::infinity()};
    double inlier_ratio{0.0};
  };

  struct AlignmentResult
  {
    bool accepted{false};
    Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  };

  void initialize()
  {
    const auto ref_directory = resolve_directory(get_parameter("ref_directory").as_string());
    const auto eval_directory = resolve_directory(get_parameter("eval_directory").as_string());
    const auto ref_files = list_pcd_files(ref_directory);
    const auto eval_files = list_pcd_files(eval_directory);

    if (ref_files.size() != 1) {
      RCLCPP_ERROR(
        get_logger(), "Reference directory must contain exactly one .pcd file: '%s' (%zu found)",
        ref_directory.c_str(), ref_files.size());
      return;
    }
    if (eval_files.empty()) {
      RCLCPP_WARN(
        get_logger(), "No evaluation .pcd files found in '%s'; publishing reference only",
        eval_directory.c_str());
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    ref_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pcd_compare/ref/points", qos);
    const auto reference = load_cloud(ref_files.front());
    if (!reference) {
      return;
    }
    publish_cloud(ref_files.front().filename().string(), reference, ref_publisher_);

    const bool alignment_enabled = get_parameter("alignment.enabled").as_bool();
    Cloud::Ptr downsampled_reference;
    if (alignment_enabled) {
      const double leaf_size = get_parameter("alignment.voxel_leaf_size").as_double();
      if (leaf_size <= 0.0) {
        RCLCPP_ERROR(get_logger(), "alignment.voxel_leaf_size must be greater than zero");
        return;
      }
      downsampled_reference = downsample(reference, static_cast<float>(leaf_size));
      RCLCPP_INFO(
        get_logger(), "Reference: %s (%zu points, %zu after downsampling)",
        ref_files.front().filename().c_str(), reference->size(), downsampled_reference->size());
    }

    for (std::size_t index = 0; index < eval_files.size(); ++index) {
      const auto topic =
        "/pcd_compare/eval/map_" + std::to_string(index) + "/points";
      const auto publisher = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
      eval_publishers_.push_back(publisher);

      auto evaluation = load_cloud(eval_files[index]);
      if (!evaluation) {
        continue;
      }
      if (alignment_enabled) {
        const auto result = align_to_reference(
          eval_files[index].filename().string(), evaluation, downsampled_reference);
        if (result.accepted) {
          apply_transform(evaluation, result.transform);
        }
      }
      publish_cloud(eval_files[index].filename().string(), evaluation, publisher);
    }
  }

  std::filesystem::path resolve_directory(const std::string & configured_path) const
  {
    std::filesystem::path path(configured_path);
    if (path.is_relative()) {
      path = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("pcd_compare")) / path;
    }
    return path.lexically_normal();
  }

  std::vector<std::filesystem::path> list_pcd_files(
    const std::filesystem::path & directory) const
  {
    std::vector<std::filesystem::path> files;
    std::error_code error;
    if (!std::filesystem::is_directory(directory, error)) {
      RCLCPP_ERROR(get_logger(), "PCD directory does not exist: '%s'", directory.c_str());
      return files;
    }
    for (const auto & entry : std::filesystem::directory_iterator(directory)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      std::string extension = entry.path().extension().string();
      std::transform(
        extension.begin(), extension.end(), extension.begin(),
        [](unsigned char value) {return static_cast<char>(std::tolower(value));});
      if (extension == ".pcd") {
        files.push_back(entry.path());
      }
    }
    std::sort(files.begin(), files.end());
    return files;
  }

  Cloud::Ptr load_cloud(const std::filesystem::path & path)
  {
    auto cloud = Cloud::Ptr(new Cloud);
    if (pcl::io::loadPCDFile<Point>(path.string(), *cloud) < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot load PCD: '%s'", path.c_str());
      return nullptr;
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %s: %zu points", path.filename().c_str(), cloud->size());
    return cloud;
  }

  static void apply_transform(Cloud::Ptr & cloud, const Eigen::Matrix4f & transform)
  {
    auto transformed = Cloud::Ptr(new Cloud);
    pcl::transformPointCloud(*cloud, *transformed, transform);
    cloud = transformed;
  }

  static Cloud::Ptr downsample(const Cloud::ConstPtr & input, float leaf_size)
  {
    auto output = Cloud::Ptr(new Cloud);
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*output);
    return output;
  }

  static Score evaluate_alignment(
    const Cloud::ConstPtr & source,
    const Cloud::ConstPtr & target,
    const Eigen::Matrix4f & transform,
    double max_distance)
  {
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform);
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    std::vector<int> nearest_index(1);
    std::vector<float> squared_distance(1);
    const double max_squared_distance = max_distance * max_distance;
    double squared_error_sum = 0.0;
    std::size_t inlier_count = 0;

    for (const auto & point : transformed) {
      if (tree.nearestKSearch(point, 1, nearest_index, squared_distance) == 1 &&
        squared_distance[0] <= max_squared_distance)
      {
        squared_error_sum += squared_distance[0];
        ++inlier_count;
      }
    }

    Score score;
    if (inlier_count > 0) {
      score.mean_squared_error = squared_error_sum / static_cast<double>(inlier_count);
    }
    if (!source->empty()) {
      score.inlier_ratio = static_cast<double>(inlier_count) / source->size();
    }
    return score;
  }

  AlignmentResult align_to_reference(
    const std::string & filename,
    const Cloud::ConstPtr & evaluation,
    const Cloud::ConstPtr & reference)
  {
    AlignmentResult result;
    const double leaf_size = get_parameter("alignment.voxel_leaf_size").as_double();
    const double max_distance =
      get_parameter("alignment.max_correspondence_distance").as_double();
    const int max_iterations =
      static_cast<int>(get_parameter("alignment.max_iterations").as_int());
    const double transformation_epsilon =
      get_parameter("alignment.transformation_epsilon").as_double();
    const double fitness_epsilon = get_parameter("alignment.fitness_epsilon").as_double();
    const double min_inlier_ratio = get_parameter("alignment.min_inlier_ratio").as_double();
    const double max_inlier_drop =
      get_parameter("alignment.max_inlier_ratio_drop").as_double();
    const double min_relative_improvement =
      get_parameter("alignment.min_relative_mse_improvement").as_double();
    const std::string initial_guess_mode =
      get_parameter("alignment.initial_guess").as_string();

    if (max_distance <= 0.0 || max_iterations <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid alignment distance or iteration parameter");
      return result;
    }

    const auto source = downsample(evaluation, static_cast<float>(leaf_size));
    if (source->empty() || reference->empty()) {
      RCLCPP_ERROR(
        get_logger(), "Alignment failed for %s: downsampled cloud is empty",
        filename.c_str());
      return result;
    }

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    if (initial_guess_mode == "centroid") {
      Eigen::Vector4f reference_centroid;
      Eigen::Vector4f source_centroid;
      pcl::compute3DCentroid(*reference, reference_centroid);
      pcl::compute3DCentroid(*source, source_centroid);
      initial_guess.block<3, 1>(0, 3) =
        (reference_centroid - source_centroid).head<3>();
    } else if (initial_guess_mode != "identity") {
      RCLCPP_ERROR(
        get_logger(), "Unknown alignment.initial_guess '%s'; use identity or centroid",
        initial_guess_mode.c_str());
      return result;
    }

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(source);
    icp.setInputTarget(reference);
    icp.setMaximumIterations(max_iterations);
    icp.setMaxCorrespondenceDistance(max_distance);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(fitness_epsilon);
    icp.setTransformationEstimation(
      pcl::make_shared<pcd_compare::TransformationEstimationXyzYaw<Point, Point>>());

    Cloud aligned;
    icp.align(aligned, initial_guess);
    const Eigen::Matrix4f transform = icp.getFinalTransformation();
    const Score before = evaluate_alignment(
      source, reference, Eigen::Matrix4f::Identity(), max_distance);
    const Score after = evaluate_alignment(source, reference, transform, max_distance);
    const double relative_improvement = std::isfinite(before.mean_squared_error) &&
      before.mean_squared_error > 0.0 ?
      (before.mean_squared_error - after.mean_squared_error) / before.mean_squared_error :
      0.0;
    const bool inlier_ratio_valid = after.inlier_ratio >= min_inlier_ratio &&
      after.inlier_ratio + max_inlier_drop >= before.inlier_ratio;
    result.accepted = icp.hasConverged() && std::isfinite(after.mean_squared_error) &&
      relative_improvement >= min_relative_improvement && inlier_ratio_valid;
    result.transform = transform;

    const double yaw = std::atan2(transform(1, 0), transform(0, 0));
    RCLCPP_INFO(
      get_logger(),
      "%s ICP %s: MSE %.6f -> %.6f m^2 (%.1f%% better), inliers %.2f%% -> %.2f%%",
      filename.c_str(), result.accepted ? "accepted" : "rejected",
      before.mean_squared_error, after.mean_squared_error, relative_improvement * 100.0,
      before.inlier_ratio * 100.0, after.inlier_ratio * 100.0);
    RCLCPP_INFO(
      get_logger(), "%s transform: x=%.6f y=%.6f z=%.6f yaw=%.6f rad",
      filename.c_str(), transform(0, 3), transform(1, 3), transform(2, 3), yaw);

    if (!result.accepted) {
      RCLCPP_WARN(
        get_logger(), "%s is published without automatic alignment",
        filename.c_str());
    }
    return result;
  }

  void publish_cloud(
    const std::string & filename,
    const Cloud::ConstPtr & cloud,
    const Publisher & publisher)
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header.frame_id = get_parameter("frame_id").as_string();
    message.header.stamp = now();
    publisher->publish(message);
    RCLCPP_INFO(get_logger(), "Published %s: %zu points", filename.c_str(), cloud->size());
  }

  Publisher ref_publisher_;
  std::vector<Publisher> eval_publishers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
