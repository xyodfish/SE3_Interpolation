#include "rigidBodyMotion.h"

inline Eigen::Vector3d angleAxisToVector3d(Eigen::AngleAxisd rotation_vector) {
    Eigen::Vector3d vector;
    vector = rotation_vector.axis();
    vector *= rotation_vector.angle();

    return vector;
}

std::vector<double> calSE3(const std::vector<double>& translate, Eigen::Quaterniond quat) {
    // 检查输入参数有效性
    if (translate.size() != 3) {
        throw std::invalid_argument("Translation vector must have 3 elements");
    }

    // 初始化结果向量，共6个元素：3个平移 + 3个轴角
    std::vector<double> se3(6);

    // 前3个元素为平移部分
    se3[0] = translate[0];
    se3[1] = translate[1];
    se3[2] = translate[2];

    // 将四元数转换为轴角表示
    // 轴角的表示为：旋转轴 * 旋转角度
    Eigen::AngleAxisd rotation_vector(quat);
    Eigen::Vector3d vector = angleAxisToVector3d(rotation_vector);

    se3.at(3) = vector(0);
    se3.at(4) = vector(1);
    se3.at(5) = vector(2);

    return se3;
}

Eigen::Matrix<double, 6, 1> vectorToEigen(std::vector<double> arg) {
    return (Eigen::Map<Eigen::Matrix<double, 6, 1>>(arg.data()));
}

Eigen::Quaterniond getQuaternionFromTcpPose(const std::vector<double>& pose) {
    Eigen::Vector3d angle;
    angle(0) = pose.at(3);
    angle(1) = pose.at(4);
    angle(2) = pose.at(5);

    auto rv = vector3dToAngleAxis(angle);

    Eigen::Quaterniond quaternion(rv);

    return quaternion;
}

Eigen::Quaterniond getQuaternionFromTcpPose(const Eigen::Matrix<double, 6, 1>& pose) {
    Eigen::Vector3d angle;
    angle(0) = pose(3);
    angle(1) = pose(4);
    angle(2) = pose(5);

    auto rv = vector3dToAngleAxis(angle);
    Eigen::Quaterniond quaternion(rv);

    return quaternion;
}

void setVector6dPoseFromQuaternion(Vector6d& pose, const Eigen::Quaterniond& q) {
    Eigen::AngleAxisd rotation_vector(q);
    Eigen::Vector3d vector = angleAxisToVector3d(rotation_vector);

    pose[3] = vector(0);
    pose[4] = vector(1);
    pose[5] = vector(2);
}

Vector6d vector6dPoseInterpolation(const Vector6d& from, const Vector6d& to, double t) {
    t                         = std::min(std::max(0.0, t), 1.0);
    Vector6d result           = from;
    Eigen::Quaterniond q_from = getQuaternionFromTcpPose(from);
    Eigen::Quaterniond q_to   = getQuaternionFromTcpPose(to);

    for (int i = 0; i < 3; i++)
        result[i] += (to[i] - from[i]) * t;

    Eigen::Quaterniond q_temp = q_from.slerp(t, q_to);

    setVector6dPoseFromQuaternion(result, q_temp);
    return result;
}

std::vector<double> vector6dToStdPose(Vector6d pose) {
    return std::vector<double>(&pose[0], pose.data() + pose.cols() * pose.rows());
}

Eigen::Matrix3d get_rot(const double& angle, const Eigen::Vector3d& axis, double eps) {
    if (std::abs(angle) <= eps)
        return Eigen::Matrix3d::Identity();

    double norm = axis.norm();
    if (norm <= eps)
        return Eigen::Matrix3d::Identity();

    Eigen::AngleAxisd rot(angle, axis / norm);
    return rot.toRotationMatrix();
};

Eigen::Isometry3d pose2Homogeneous(const Eigen::Matrix<double, 6, 1>& pose) {
    Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
    frame.rotate(get_rot(pose.segment(3, 3).norm(), pose.segment(3, 3)));
    frame.pretranslate(Eigen::Vector3d(pose(0), pose(1), pose(2)));
    return frame;
}

Eigen::Isometry3d pose2Homogeneous(const std::vector<double>& pose) {
    return pose2Homogeneous(vectorToEigen(pose));
}

Eigen::AngleAxisd vector3dToAngleAxis(Eigen::Vector3d vector) {
    Eigen::AngleAxisd rotation_vector(vector.norm(), vector.normalized());
    return rotation_vector;
}

template <typename T>
T vectorToEigen(std::vector<double> arg) {
    // 使用Eigen::Map将std::vector映射到Eigen类型
    return Eigen::Map<T>(arg.data());
}