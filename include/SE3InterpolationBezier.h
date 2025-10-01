#ifndef __SE3_INTERPOLATION_BEZIER_H__
#define __SE3_INTERPOLATION_BEZIER_H__

#include <memory>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "SE3InterpolationInterface.h"

namespace SE3::Iterpolation {
    class SE3InterpolationBezier : public SE3InterpolationInterface {
       public:
        explicit SE3InterpolationBezier() = default;
        explicit SE3InterpolationBezier(const std::vector<std::vector<double>>& trajPoints, const std::vector<double>& timeSeries);

        ~SE3InterpolationBezier() override = default;

        std::vector<double> getInterpolationVector(size_t index, double t);

        virtual void calculateResultsSeries() override;

        class SingleTrajectory {
           public:
            SingleTrajectory(const std::vector<double>& p0, const std::vector<double>& p1, const std::vector<double>& p2,
                             const std::vector<double>& p3) {

                p0_ = vectorToEigen<Vector3d>(p0);
                p1_ = vectorToEigen<Vector3d>(p1);
                p2_ = vectorToEigen<Vector3d>(p2);
                p3_ = vectorToEigen<Vector3d>(p3);

                q0_ = getQuaternionFromTcpPose(p0);
                q1_ = getQuaternionFromTcpPose(p0);
                q2_ = getQuaternionFromTcpPose(p1);
                q3_ = getQuaternionFromTcpPose(p2);
            }

            ~SingleTrajectory() = default;

            /// @brief
            /// @param t
            /// @return
            std::vector<double> getInterpolationVector(double t);

            /// @brief
            /// @param t
            /// @return
            Eigen::Isometry3d getInterpolation(double t);

            Eigen::Quaterniond bezierInterpolation(double t);

           private:
            Vector3d p0_, p1_, p2_, p3_;
            Eigen::Quaterniond q0_, q1_, q2_, q3_, s1_, s2_;

            std::vector<double> getBezierPos(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2, const Vector3d& p3, double t);
        };

       private:
        std::vector<std::shared_ptr<SingleTrajectory>> singleTrajectories_;

        std::vector<std::vector<double>> trajPoints_;

        std::vector<Eigen::Quaterniond> controlPoints_;

        std::vector<Eigen::Quaterniond> qSeries_;

        std::vector<double> timeSeries_;

        size_t trajSize_, timeSeriesSize_;
    };
}  // namespace SE3::Iterpolation

#endif