#pragma once
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"

const double epsilon = 1e-6;
using Vector6d       = Eigen::Matrix<double, 6, 1>;

std::vector<double> calSE3(const std::vector<double>& translate, Eigen::Quaterniond quat);
Eigen::Matrix<double, 6, 1> vectorToEigen(std::vector<double> arg);
Eigen::Quaterniond getQuaternionFromTcpPose(const std::vector<double>& pose);
void setVector6dPoseFromQuaternion(Vector6d& pose, const Eigen::Quaterniond& q);
Vector6d vector6dPoseInterpolation(const Vector6d& from, const Vector6d& to, double t);
std::vector<double> vector6dToStdPose(Vector6d pose);
Eigen::Isometry3d pose2Homogeneous(const Eigen::Matrix<double, 6, 1>& pose);
Eigen::Isometry3d pose2Homogeneous(const std::vector<double>& pose);
Eigen::Matrix3d get_rot(const double& angle, const Eigen::Vector3d& axis, double eps = epsilon);
Eigen::Quaterniond getQuaternionFromTcpPose(const Eigen::Matrix<double, 6, 1>& pose);
Eigen::AngleAxisd vector3dToAngleAxis(Eigen::Vector3d vector);
template <typename T>
T vectorToEigen(std::vector<double> arg);