#include "SE3InterpolationSquad.h"
#include "lieGroupUtils.h"
#include "rigidBodyMotion.h"

namespace LPSO3 = LieGroup::SO3;
namespace LPSE3 = LieGroup::SE3;

namespace SE3::Iterpolation {
    SE3InterpolationSquad::SE3InterpolationSquad(const std::vector<std::vector<double>>& trajPoints, const std::vector<double>& timeSeries)
        : trajPoints_(trajPoints), timeSeries_(timeSeries) {

        calculateControlPoints(trajPoints_);
        calculateQuaternionSeries(trajPoints_);

        for (auto i = 0; i < trajPoints_.size() - 1; i++) {
            singleTrajectories_.push_back(
                std::make_shared<SingleTrajectory>(trajPoints_[i], trajPoints_[i + 1], controlPoints_[i], controlPoints_[i + 1]));
        }
    }

    SE3InterpolationSquad::~SE3InterpolationSquad() = default;

    void SE3InterpolationSquad::calculateQuaternionSeries(const std::vector<std::vector<double>>& trajPoints) {
        for (auto& trajPoint : trajPoints) {
            qSeries_.push_back(getQuaternionFromTcpPose(trajPoint));
        }
    }

    void SE3InterpolationSquad::calculateControlPoints(const std::vector<std::vector<double>>& trajPoints) {

        controlPoints_.resize(trajPoints.size());
        const auto num = controlPoints_.size();

        controlPoints_[0]       = getQuaternionFromTcpPose(trajPoints[0]);
        controlPoints_[num - 1] = getQuaternionFromTcpPose(trajPoints[num - 1]);

        for (auto i = 1; i < num - 1; ++i) {
            controlPoints_[i] = calSquadCtrlPoint(getQuaternionFromTcpPose(trajPoints[i - 1]), getQuaternionFromTcpPose(trajPoints[i]),
                                                  getQuaternionFromTcpPose(trajPoints[i + 1]));
        }
    }

    Eigen::Quaterniond SE3InterpolationSquad::calSquadCtrlPoint(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2,
                                                                const Eigen::Quaterniond& q3) {
        Eigen::Quaterniond m0, m1, m0_log, m1_log, m_log_sum, k, k_exp;
        Eigen::Quaterniond t_q1 = q1, t_q2 = q2, t_q3 = q3;

        if (t_q1.dot(t_q2) < 0)
            t_q2 = LPSO3::qReverse(t_q2);
        if (t_q1.dot(t_q3) < 0)
            t_q3 = LPSO3::qReverse(t_q3);

        auto q1_conj = t_q1.conjugate();
        auto q2_conj = t_q2.conjugate();
        m0           = q2_conj * t_q1;
        m1           = q1_conj * t_q3;

        // k = (log(m0)-log(m1))/4;
        m0_log = LPSO3::qLog(m0);
        m1_log = LPSO3::qLog(m1);

        m_log_sum = LPSO3::qSubstraction(m0_log, m1_log);
        k         = m_log_sum.coeffs() * 0.25;
        return LPSO3::qExp(k);
    }

    void SE3InterpolationSquad::calculateResultsSeries() {

        resultSeries_.clear();
        const auto num = trajPoints_.size();

        for (auto i = 0; i < num - 1; ++i) {
            /// 初始点
            resultSeries_.push_back(trajPoints_[i]);

            for (double j = 0.0; j < 1.0; j += timeSeries_[i]) {

                resultSeries_.push_back(
                    calSE3(LPSE3::posLinearInterpolation(trajPoints_[i], trajPoints_[i + 1], j), singleTrajectories_[i]->qSquad(j)));
            }

            resultSeries_.push_back(calSE3(trajPoints_[i + 1], getQuaternionFromTcpPose(trajPoints_[i + 1])));
        }
    }

    std::vector<double> SE3InterpolationSquad::getInterpolationVector(size_t index, double t) {

        if (index > trajPoints_.size() - 1) {
            throw std::invalid_argument("The index is out of range");
        }

        return singleTrajectories_[index]->interpolateSquad(t);
    }

    Eigen::Quaterniond SE3InterpolationSquad::SingleTrajectory::qSquad(double t) {
        return LPSO3::qSlerp(LPSO3::qSlerp(qf_, qt_, t), LPSO3::qSlerp(s1_, s2_, t), 2 * t * (1 - t));
    }

    std::vector<double> SE3InterpolationSquad::SingleTrajectory::interpolateSquad(double t) {
        return calSE3(LPSE3::posLinearInterpolation(sPose_, gPose_, t), qSquad(t));
    }

}  // namespace SE3::Iterpolation