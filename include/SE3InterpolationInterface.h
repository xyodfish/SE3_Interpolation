#ifndef __SE3_INTERPOLATION_INTERFACE_H__
#define __SE3_INTERPOLATION_INTERFACE_H__

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "rigidBodyMotion.h"

namespace SE3::Iterpolation {
    class SE3InterpolationInterface {

       public:
        using Vector6d = Eigen::Matrix<double, 6, 1>;
        using Vector3d = Eigen::Vector3d;

        explicit SE3InterpolationInterface() = default;
        explicit SE3InterpolationInterface(const std::vector<std::vector<double>>& trajPoints, const std::vector<double>& timeSeries){};

        virtual ~SE3InterpolationInterface() = default;

        /// @brief
        /// @param t
        /// @return
        virtual std::vector<double> getInterpolationVector(double t) {
            t = std::min(std::max(0.0, t), 1.0);

            if (resultSeries_.empty()) {
                calculateResultsSeries();
            }

            return resultSeries_[static_cast<size_t>(t * resultSeries_.size())];
        }

        virtual Eigen::Isometry3d getInterpolation(double t) { return pose2Homogeneous(getInterpolationVector(t)); }

        /// @brief
        virtual void calculateResultsSeries() = 0;  // 可用于外部调用 计算全局序列

        std::vector<std::vector<double>> getTrajs() const { return resultSeries_; }

       protected:
        std::vector<std::vector<double>> resultSeries_;
    };
}  // namespace SE3::Iterpolation

#endif