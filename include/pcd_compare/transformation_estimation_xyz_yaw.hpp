#pragma once

#include <cmath>

#include <Eigen/Core>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_2D.h>

namespace pcd_compare
{

// PCL's TransformationEstimation2D estimates x/y/yaw and intentionally ignores z.
// This variant keeps the same planar rotation model while estimating z translation
// from the correspondence centroids. Roll and pitch always remain zero.
template<typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationEstimationXyzYaw
  : public pcl::registration::TransformationEstimation2D<
    PointSource, PointTarget, Scalar>
{
public:
  using Base = pcl::registration::TransformationEstimation2D<
    PointSource, PointTarget, Scalar>;
  using Matrix4 = typename Base::Matrix4;

  void estimateRigidTransformation(
    const pcl::PointCloud<PointSource> & source,
    const pcl::PointCloud<PointTarget> & target,
    const pcl::Correspondences & correspondences,
    Matrix4 & transform) const override
  {
    transform.setIdentity();
    if (correspondences.size() < 3) {
      return;
    }

    Eigen::Matrix<Scalar, 3, 1> source_centroid =
      Eigen::Matrix<Scalar, 3, 1>::Zero();
    Eigen::Matrix<Scalar, 3, 1> target_centroid =
      Eigen::Matrix<Scalar, 3, 1>::Zero();
    for (const auto & correspondence : correspondences) {
      source_centroid += source[correspondence.index_query].getVector3fMap()
        .template cast<Scalar>();
      target_centroid += target[correspondence.index_match].getVector3fMap()
        .template cast<Scalar>();
    }
    const Scalar count = static_cast<Scalar>(correspondences.size());
    source_centroid /= count;
    target_centroid /= count;

    Scalar numerator = 0;
    Scalar denominator = 0;
    for (const auto & correspondence : correspondences) {
      const auto source_centered =
        source[correspondence.index_query].getVector3fMap().template cast<Scalar>() -
        source_centroid;
      const auto target_centered =
        target[correspondence.index_match].getVector3fMap().template cast<Scalar>() -
        target_centroid;
      numerator += source_centered.x() * target_centered.y() -
        source_centered.y() * target_centered.x();
      denominator += source_centered.x() * target_centered.x() +
        source_centered.y() * target_centered.y();
    }

    const Scalar yaw = std::atan2(numerator, denominator);
    const Scalar cosine = std::cos(yaw);
    const Scalar sine = std::sin(yaw);
    transform(0, 0) = cosine;
    transform(0, 1) = -sine;
    transform(1, 0) = sine;
    transform(1, 1) = cosine;
    transform(0, 3) = target_centroid.x() -
      cosine * source_centroid.x() + sine * source_centroid.y();
    transform(1, 3) = target_centroid.y() -
      sine * source_centroid.x() - cosine * source_centroid.y();
    transform(2, 3) = target_centroid.z() - source_centroid.z();
  }
};

}  // namespace pcd_compare
