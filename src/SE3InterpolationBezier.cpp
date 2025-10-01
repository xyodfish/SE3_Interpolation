#include "SE3InterpolationBezier.h"
#include "lieGroupUtils.h"
#include "rigidBodyMotion.h"

namespace LPSO3 = LieGroup::SO3;
namespace LPSE3 = LieGroup::SE3;
namespace SE3::Iterpolation {

    SE3InterpolationBezier::SE3InterpolationBezier(const std::vector<std::vector<double>>& trajPoints,
                                                   const std::vector<double>& timeSeries)
        : trajPoints_(trajPoints), timeSeries_(timeSeries), trajSize_(trajPoints.size()), timeSeriesSize_(timeSeries.size()) {

        if (trajSize_ < 4 || ((trajSize_ - 3) != timeSeriesSize_)) {
            throw std::invalid_argument("The trajectory points or time series are not valid");
        }

        /// 如果轨迹点个数不能保证每4个点一组，则添加剩余的点
        /// p0 p1 p2 p3
        /// p3 p4 p5 p6
        /// 如果一共5个点，则添加 3 - （5-1） % 3 = 2个点
        /// 如果一共6个点，则添加 3 - （6-1） % 3 = 1个点
        auto leftPointNum = 3 - ((trajSize_ - 1) % 3);

        if (leftPointNum > 0) {
            auto backPoint = trajPoints_.back();
            for (auto i = 0; i < leftPointNum; ++i) {
                trajPoints_.push_back(backPoint);
            }
        }

        trajSize_ = trajPoints_.size();

        for (auto i = 0; i < trajSize_ - 3; i = i + 3) {
            singleTrajectories_.push_back(
                std::make_shared<SingleTrajectory>(trajPoints_[i], trajPoints_[i + 1], trajPoints_[i + 2], trajPoints_[i + 3]));
        }
    }

    void SE3InterpolationBezier::calculateResultsSeries() {

        resultSeries_.clear();
        const auto num = trajSize_;
        size_t sgIndex = 0;

        for (auto i = 0; i < num - 1; ++i) {
            /// 初始点
            resultSeries_.push_back(trajPoints_[i]);

            sgIndex = i % 4;

            for (double j = timeSeries_[j]; j < 1.0; j += timeSeries_[i]) {
                resultSeries_.push_back(singleTrajectories_[sgIndex]->getInterpolationVector(j));
            }

            resultSeries_.push_back(trajPoints_[i + 1]);
        }
    }

    std::vector<double> SE3InterpolationBezier::getInterpolationVector(size_t index, double t) {

        if (index > trajPoints_.size() - 1) {
            throw std::invalid_argument("The index is out of range");
        }

        return singleTrajectories_[index]->getInterpolationVector(t);
    }

    std::vector<double> SE3InterpolationBezier::SingleTrajectory::getBezierPos(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2,
                                                                               const Vector3d& p3, double t) {

        // 计算贝塞尔曲线位置
        Eigen::Vector3d retPos =
            std::pow((1 - t), 3) * p0 + 3 * t * std::pow((1 - t), 2) * p1 + 3 * std::pow(t, 2) * (1 - t) * p2 + std::pow(t, 3) * p3;

        return {retPos[0], retPos[1], retPos[2]};
    }

    std::vector<double> SE3InterpolationBezier::SingleTrajectory::getInterpolationVector(double t) {

        auto pos = getBezierPos(p0_, p1_, p2_, p3_, t);
        return calSE3(pos, bezierInterpolation(t));
    }

    Eigen::Quaterniond SE3InterpolationBezier::SingleTrajectory::bezierInterpolation(double t) {

        Eigen::Quaterniond q01, q12, q23;

        Eigen::Quaterniond q012, q123, q0123;

        q01  = LPSO3::qSlerp(q0_, q1_, t);
        q12  = LPSO3::qSlerp(q1_, q2_, t);
        q012 = LPSO3::qSlerp(q01, q12, t);

        q23  = LPSO3::qSlerp(q2_, q3_, t);
        q123 = LPSO3::qSlerp(q12, q23, t);

        q0123 = LPSO3::qSlerp(q012, q123, t);

        return q0123;
    }
}  // namespace SE3::Iterpolation