#include "lieGroupUtils.h"

namespace LieGroup {

    namespace SO3 {
        Eigen::Quaterniond qExp(const Eigen::Quaterniond& q1) {

            double a     = q1.vec().norm();
            double exp_w = std::exp(q1.w());

            if (a == 0.0) {
                return Eigen::Quaterniond(exp_w, 0, 0, 0);
            }

            Eigen::Quaterniond res;
            res.w()   = exp_w * std::cos(a);
            res.vec() = exp_w * std::sin(a) * q1.vec();

            return res;
        }

        Eigen::Quaterniond qLog(const Eigen::Quaterniond& q1) {
            Eigen::Quaterniond q = q1.normalized();

            double exp_w = q.norm();
            double w     = std::log(exp_w);
            double a     = std::acos(q.w() / exp_w);

            if (a == 0.0) {
                return Eigen::Quaterniond(w, 0.0, 0.0, 0.0);
            }

            Eigen::Quaterniond res;
            res.w()   = w;
            res.vec() = q.vec() / exp_w / (std::sin(a) / a);

            return res;
        }

        Eigen::Quaterniond qReverse(const Eigen::Quaterniond& q1) {
            Eigen::Quaterniond q2(-q1.w(), -q1.x(), -q1.y(), -q1.z());
            return q2;
        }

        Eigen::Quaterniond qSubstraction(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
            return Eigen::Quaterniond(q1.w() - q2.w(), q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z());
        }

        Eigen::Quaterniond qPlus(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
            return Eigen::Quaterniond(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
        }

        Eigen::Quaterniond qMultiply(const Eigen::Quaterniond& q1, double scalar) {
            return Eigen::Quaterniond(q1.w() * scalar, q1.x() * scalar, q1.y() * scalar, q1.z() * scalar);
        }

        Eigen::Quaterniond qSlerp(const Eigen::Quaterniond& qf, const Eigen::Quaterniond& qt, double t) {
            t = std::min(std::max(t, 0.0), 1.0);
            return qf.slerp(t, qt);
        }

        /// @brief calculate the dot product of two quaternions
        /// @param q1
        /// @param q2
        /// @return
        double qDotProduct(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
            return q1.w() * q2.w() + q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z();
        }

        Eigen::Quaterniond qDoubleProjection(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
            Eigen::Quaterniond q;
            q = q2.coeffs() * 2 * qDotProduct(q1, q2);
            return qSubstraction(q, q1);
        }

        Eigen::Quaterniond biSect(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
            auto q        = qPlus(q1, q2);
            double length = 1.0 / std::sqrt(q.squaredNorm());
            return qMultiply(q, length);
        }
    }  // namespace SO3

    namespace SE3 {

        std::vector<double> posLinearInterpolation(const std::vector<double>& from, const std::vector<double>& to,
                                                   double t) {
            std::vector<double> res = from;
            t                       = std::min(std::max(0.0, t), 1.0);

            for (size_t i = 0; i < 3; i++) {
                res[i] += (to[i] - from[i]) * t;
            }

            return res;
        }

        std::vector<double> posLinearInterpolation(const Vector6d& from, const Vector6d& to, double t) {
            std::vector<double> res(6);
            t = std::min(std::max(0.0, t), 1.0);

            for (size_t i = 0; i < 3; i++) {
                res[i] = from[i] + (to[i] - from[i]) * t;
            }

            return res;
        }

    }  // namespace SE3

}  // namespace LieGroup