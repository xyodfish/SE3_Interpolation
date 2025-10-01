#ifndef _SE3_TRAJECTORY_INTERPOLATOR_H_
#define _SE3_TRAJECTORY_INTERPOLATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "SE3InterpolationBezier.h"
#include "SE3InterpolationSlerp.h"
#include "SE3InterpolationSquad.h"

namespace SE3::Iterpolation {

    class SE3TrajectoryInterpolator {
       public:
        SE3TrajectoryInterpolator(const std::vector<std::vector<double>>& trajectory, const std::vector<double>& timeSeries,
                                  const std::string& interTypes);
        ~SE3TrajectoryInterpolator() = default;

        Eigen::Isometry3d getInterpolation(double t);

        std::vector<double> getInterpolationVector(double t);

        /// @brief get the whole trajectory constructed by the interpolators
        /// @return
        std::vector<std::vector<double>> getTrajs();

        /// @brief get the trajectory at index in the piecewise trajectory
        /// @param index
        /// @return
        std::vector<std::vector<double>> getTrajs(size_t index);

        enum class INTER_TYPE_ENUM { SLERP, SQUAD, BEZIER, INVALID };

       private:
        std::vector<std::vector<double>> trajectory_;
        std::vector<double> timeSeries_;  // 轨迹每段插值点的时间序列
        std::string interType_;
        std::shared_ptr<SE3InterpolationInterface> interpolator_;

        size_t trajSize_, timeSeriesSize_;

       private:
        /// @brief generate the pose interpolator in SE3 space
        void generateInterpolators(INTER_TYPE_ENUM interType);

        /// @brief
        void generateSlerpInterpolators();

        /// @brief
        void generateSquadInterpolators();

        /// @brief
        void generateBezierInterpolators();

        const size_t MIN_SQUAD_POINTS_SIZE = 3, MIN_BEZIER_POINTS_SIZE = 4;
    };
}  // namespace SE3::Iterpolation

#endif