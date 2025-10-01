#ifndef __SE3_INTERPOLATION_SLERP_H__
#define __SE3_INTERPOLATION_SLERP_H__

#include <memory>
#include "SE3InterpolationInterface.h"

namespace SE3::Iterpolation {

    class SE3InterpolationSlerp : public SE3InterpolationInterface {
       public:
        explicit SE3InterpolationSlerp() = default;
        explicit SE3InterpolationSlerp(const std::vector<std::vector<double>>& trajPoints, const std::vector<double>& timeSeries);
        ~SE3InterpolationSlerp() override;

        std::vector<double> getInterpolationVector(size_t index, double t);

        virtual void calculateResultsSeries() override;

        class SingleTrajectory {
           public:
            explicit SingleTrajectory(const std::vector<double>& pf, const std::vector<double>& pt) {
                sPose_ = vectorToEigen(pf);
                gPose_ = vectorToEigen(pt);
                qf_    = getQuaternionFromTcpPose(pf);
                qt_    = getQuaternionFromTcpPose(pt);
            }

            ~SingleTrajectory() = default;

            Eigen::Isometry3d getInterpolation(double t);
            std::vector<double> getInterpolationVector(double t);

           private:
            Vector6d sPose_, gPose_;
            Eigen::Quaterniond qf_, qt_;
        };

       private:
        std::vector<std::vector<double>> trajPoints_;
        std::vector<double> timeSeries_;
        std::vector<std::vector<double>> resultSeries_;

        std::vector<std::shared_ptr<SingleTrajectory>> singleTrajectories_;
    };

}  // namespace SE3::Iterpolation

#endif