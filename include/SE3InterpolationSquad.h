#ifndef __SE3_INTERPOLATION_H__
#define __SE3_INTERPOLATION_H__

#include <memory>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "SE3InterpolationInterface.h"

namespace SE3::Iterpolation {
    class SE3InterpolationSquad : public SE3InterpolationInterface {
       public:
        using Vector6d = Eigen::Matrix<double, 6, 1>;

        explicit SE3InterpolationSquad() = default;

        explicit SE3InterpolationSquad(const std::vector<std::vector<double>>& trajPoints, const std::vector<double>& timeSeries);

        ~SE3InterpolationSquad() override;

        std::vector<double> getInterpolationVector(size_t index, double t);

        virtual void calculateResultsSeries() override;

        class SingleTrajectory {
           public:
            SingleTrajectory(const std::vector<double>& pf, const std::vector<double>& pt, const Eigen::Quaterniond& s1,
                             const Eigen::Quaterniond& s2) {

                sPose_ = vectorToEigen(pf);
                gPose_ = vectorToEigen(pt);
                qf_    = getQuaternionFromTcpPose(pf);
                qt_    = getQuaternionFromTcpPose(pt);

                setControlPoints(s1, s2);
            }

            /// @brief
            /// @param t
            /// @return
            std::vector<double> getInterpolationVector(double t);

            /// @brief
            /// @param t
            /// @return
            Eigen::Isometry3d getInterpolation(double t);

            /// @brief
            /// @param t
            /// @return
            Eigen::Quaterniond qSquad(double t);

            std::vector<double> interpolateSquad(double t);

           private:
            void setControlPoints(const Eigen::Quaterniond& s1, const Eigen::Quaterniond& s2) {
                s1_ = s1;
                s2_ = s2;
            }

            void setSgPoints(const std::vector<double>& pf, const std::vector<double>& pt);

            Vector6d sPose_, gPose_;
            Eigen::Quaterniond qf_, qt_, s1_, s2_;
        };

        std::vector<std::shared_ptr<SingleTrajectory>> singleTrajectories_;

       private:
        Eigen::Quaterniond calSquadCtrlPoint(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q3);

        std::vector<std::vector<double>> trajPoints_;

        std::vector<Eigen::Quaterniond> controlPoints_;

        std::vector<Eigen::Quaterniond> qSeries_;

        std::vector<double> timeSeries_;

        void calculateControlPoints(const std::vector<std::vector<double>>& trajPoints);
        void calculateQuaternionSeries(const std::vector<std::vector<double>>& trajPoints);
    };
}  // namespace SE3::Iterpolation

#endif