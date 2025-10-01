# SE3_Interpolation
This is a simple implementation of SE3 interpolation.
you can build it with
```
mkdir build
cd build 
make -j
```

This repo is given a c++ class named 
```c++
SE3::Iterpolation::SE3TrajectoryInterpolator
```
example:
```c++
std::vector<std::vector<double>> trajectory; // input discrete trajectory point (x,y,z,aax, aay, aaz) 其中后三个旋转表示为轴角
std::vector<double> timeSeries; // input discrete time series [0.01,0.01,0.01,0.01]
std::string interType; // input interpolation type {SLERP, SQUAD, BEZIER, INVALID}
SE3::Iterpolation::SE3TrajectoryInterpolator interpolator(trajectory, timeSeries, interType);

/// @brief get the whole trajectory constructed by the interpolator
std::vector<std::vector<double>> interpolatedTrajectory = interpolator.getTrajs();

// get the interpolated trajectory point at time t
double t;
std::vector<double> interpolatedTrajPoint = interpolator.getInterpolation(t);
```

This repo may have some bugs, i will fix them.