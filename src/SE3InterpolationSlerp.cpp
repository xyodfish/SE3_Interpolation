#include "SE3InterpolationSlerp.h"

namespace SE3::Iterpolation {
    SE3InterpolationSlerp::SE3InterpolationSlerp(const std::vector<std::vector<double>>& trajPoints, const std::vector<double>& timeSeries)

        : trajPoints_(trajPoints), timeSeries_(timeSeries) {

        for (auto i = 0; i < trajPoints_.size() - 1; i++) {
            singleTrajectories_.push_back(std::make_shared<SingleTrajectory>(trajPoints_[i], trajPoints_[i + 1]));
        }
    }

    SE3InterpolationSlerp::~SE3InterpolationSlerp() = default;

    void SE3InterpolationSlerp::calculateResultsSeries() {
        resultSeries_.clear();
        for (auto i = 0; i < singleTrajectories_.size(); i++) {

            // for (auto j = 0; j < 1.0 + timeSeries_[i]; j += timeSeries_[i]) {
            //     resultSeries_.push_back(singleTrajectories_[i]->getInterpolationVector(j));
            // }

            // 修正循环逻辑：在[0,1]区间内按照timeSeries_[i]步长采样
            for (double t = 0.0; t <= 1.0; t += timeSeries_[i]) {
                // 确保t不会超过1.0
                double clamped_t = std::min(t, 1.0);
                resultSeries_.push_back(singleTrajectories_[i]->getInterpolationVector(clamped_t));

                // 如果已经到达1.0，则退出循环
                if (clamped_t >= 1.0)
                    break;
            }
        }
    }

    std::vector<double> SE3InterpolationSlerp::getInterpolationVector(size_t index, double t) {

        if (index > trajPoints_.size() - 1) {
            throw std::invalid_argument("The index is out of range");
        }

        return singleTrajectories_[index]->getInterpolationVector(t);
    }

    std::vector<double> SE3InterpolationSlerp::SingleTrajectory::getInterpolationVector(double t) {
        return vector6dToStdPose(vector6dPoseInterpolation(sPose_, gPose_, t));
    }

    Eigen::Isometry3d SE3InterpolationSlerp::SingleTrajectory::getInterpolation(double t) {
        return pose2Homogeneous(getInterpolationVector(t));
    }

}  // namespace SE3::Iterpolation