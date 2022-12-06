//
// Created by lihao on 2022/10/3.
//

#ifndef IMU_MEASUREMENT_PACKAGE_H
#define IMU_MEASUREMENT_PACKAGE_H

#include <iostream>
#include <iomanip>
#include "Eigen/Dense"


class MeasurementPackage {
public:
    double timestamp_{};

    Eigen::Vector3d gyro_;
    Eigen::Vector3d acc_;

    MeasurementPackage() : timestamp_(0), gyro_(Eigen::Vector3d::Zero()), acc_(Eigen::Vector3d::Zero())
    {}

    friend std::ostream& operator<<(std::ostream& cout, MeasurementPackage& measurement)
    {
        cout << std::fixed << std::setprecision(6) << measurement.timestamp_
        << " " << std::fixed << std::setprecision(6) << measurement.gyro_.transpose()
        << " " << std::fixed << std::setprecision(6) << measurement.acc_.transpose();
        return cout;
    }

};


#endif //IMU_MEASUREMENT_PACKAGE_H
