//
// Created by lihao on 2022/10/5.
//

#ifndef IMU_INS_FILTER_H
#define IMU_INS_FILTER_H


#include <iostream>

#include "Eigen/Dense"
#include "interface/measurement_package.h"
#include "interface/ground_truth_package.h"


#define PI  double(EIGEN_PI)
#define EPSILON 1e-6
#define EARTH_SEMI_MAJOR_AXIS 6.3781370e6
#define EARTH_SEMI_MINOR_AXIS 6356752.3142
#define EARTH_ECCENTRICITY sqrt(pow(EARTH_SEMI_MAJOR_AXIS, 2.0) - pow(EARTH_SEMI_MINOR_AXIS, 2.0)) / EARTH_SEMI_MAJOR_AXIS
#define EARTH_ROTATION_VELOCITY 7.292115e-5

#define degree2rad(x)   x * PI / 180.0
#define rad2degree(x)   x * 180.0 / PI


Eigen::Vector3d Quaternion2EulerAngles(const Eigen::Quaterniond& quaternion);
Eigen::Quaterniond EulerAngles2Quaternion(const Eigen::Vector3d& eulerAngles);


class InsFilter {
public:
    InsFilter(bool use_extrapolation=true);

    void Initialize(GroundTruthPackage& state);
    void Initialize(double timestamp, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& att);
    bool IsInitialized();

    void Update(MeasurementPackage measurement);
    void Update(double timestamp, Eigen::Vector3d& gyro, Eigen::Vector3d& acc);
    GroundTruthPackage GetState();
    Eigen::Vector3d GetPosition();
    Eigen::Vector3d GetVelocity();
    Eigen::Vector3d GetAttitude();

private:

    void EstimateAttitude();
    void EstimateVelocity();
    void EstimatePosition();

    bool is_initialized_;

    // last state
    double last_timestamp_;
    Eigen::Vector3d last_gyro_;
    Eigen::Vector3d last_acc_;
    Eigen::Vector3d last_pos_;
    Eigen::Vector3d last_vel_;
    Eigen::Quaterniond last_att_;

    // current state
    double timestamp_;
    Eigen::Vector3d gyro_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Quaterniond att_;

    // middle state
    bool use_extrapolation_;
    double delta_t_;
    Eigen::Vector3d avg_pos_;
    Eigen::Vector3d avg_vel_;
    Eigen::Vector3d omega_en_;
    Eigen::Vector3d omega_ie_;
    Eigen::Vector3d zeta_;
};


#endif //IMU_INS_FILTER_H
