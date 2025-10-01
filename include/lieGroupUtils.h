#ifndef __LIE_GROUP_UTILS_H__
#define __LIE_GROUP_UTILS_H__

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace LieGroup {
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    namespace SO3 {
        Eigen::Quaterniond qExp(const Eigen::Quaterniond& q1);
        Eigen::Quaterniond qLog(const Eigen::Quaterniond& q1);
        Eigen::Quaterniond qReverse(const Eigen::Quaterniond& q1);
        Eigen::Quaterniond qSubstraction(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
        Eigen::Quaterniond qSlerp(const Eigen::Quaterniond& qf, const Eigen::Quaterniond& qt, double t);

        double qDotProduct(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
        Eigen::Quaterniond qDoubleProjection(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
        Eigen::Quaterniond biSect(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);

    }  // namespace SO3

    namespace SE3 {
        std::vector<double> posLinearInterpolation(const std::vector<double>& from, const std::vector<double>& to,
                                                   double t);
        std::vector<double> posLinearInterpolation(const Vector6d& from, const Vector6d& to, double t);

    }  // namespace SE3

}  // namespace LieGroup

#endif