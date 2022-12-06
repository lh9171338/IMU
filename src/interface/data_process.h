//
// Created by lihao on 2022/10/5.
//

#ifndef IMU_DATA_PROCESS_H
#define IMU_DATA_PROCESS_H


#include <iostream>
#include <fstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "ground_truth_package.h"


bool load_measurement(const std::string& filename, std::vector<MeasurementPackage>& measurements, bool binary=false) {
    measurements.clear();
    std::ifstream fs(filename, std::ios::in | std::ios::binary);
    if (!fs.is_open()) {
        std::cout << "can't open file " << filename << std::endl;
        return false;
    }
    Eigen::Matrix<double, 7, 1> values;
    while(fs.read((char*)&values, sizeof(values))) {
        MeasurementPackage measurement;
        measurement.timestamp_ = values[0];
        measurement.gyro_ = values.segment<3>(1);
        measurement.acc_ = values.segment<3>(4);
        measurements.push_back(measurement);
    }
    fs.close();

    return true;
}

bool load_ground_truth(const std::string& filename, std::vector<GroundTruthPackage>& ground_truths, bool binary=false) {
    ground_truths.clear();
    std::ifstream fs(filename, std::ios::in | std::ios::binary);
    if (!fs.is_open()) {
        std::cout << "can't open file " << filename << std::endl;
        return false;
    }
    Eigen::Matrix<double, 10, 1> values;
    while(fs.read((char*)&values, sizeof(values))) {
        GroundTruthPackage ground_truth;
        ground_truth.timestamp_ = values[0];
        ground_truth.pos_ = values.segment<3>(1);
        ground_truth.vel_ = values.segment<3>(4);
        ground_truth.att_ = values.segment<3>(7);
        ground_truths.push_back(ground_truth);
    }
    fs.close();

    return true;
}

template<class T>
bool save(const std::string& filename, std::vector<T>& results) {
    std::ofstream fs(filename, std::ios::out);
    if (!fs.is_open()) {
        std::cout << "can't open file " << filename << std::endl;
        return false;
    }
    for(auto& result : results) {
        fs << result << std::endl;
    }

    return true;
}


#endif //IMU_DATA_PROCESS_H
