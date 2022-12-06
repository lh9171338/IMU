//
// Created by lihao on 2022/10/3.
//

#ifndef IMU_GROUND_TRUTH_PACKAGE_H
#define IMU_GROUND_TRUTH_PACKAGE_H

#include <iostream>
#include <iomanip>
#include "Eigen/Dense"


class GroundTruthPackage {
public:
    double timestamp_{};

    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d att_;

    GroundTruthPackage() : timestamp_(0), pos_(Eigen::Vector3d::Zero()), vel_(Eigen::Vector3d::Zero()),
    att_(Eigen::Vector3d::Zero())
    {}

    friend std::ostream& operator<<(std::ostream& cout, GroundTruthPackage& ground_truth)
    {
        cout << std::fixed << std::setprecision(6) << ground_truth.timestamp_
        << " " << std::fixed << std::setprecision(6) << ground_truth.pos_.transpose()
        << " " << std::fixed << std::setprecision(6) << ground_truth.vel_.transpose()
        << " " << std::fixed << std::setprecision(6) << ground_truth.att_.transpose();
        return cout;
    }

};


#endif //IMU_GROUND_TRUTH_PACKAGE_H
