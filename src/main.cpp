//
// Created by lihao on 2022/10/5.
//

#include <iostream>
#include <vector>
#include "interface/measurement_package.h"
#include "interface/ground_truth_package.h"
#include "interface/data_process.h"
#include "filter/ins_filter.h"
#include "Eigen/Dense"


int main() {
    std::string imu_file = "../data/imu.bin";
    std::string ground_truth_file = "../data/ground_truth.bin";
    std::string result_file = "../data/result.txt";

    std::vector<MeasurementPackage> measurements;
    std::vector<GroundTruthPackage> ground_truths;
    if(!load_measurement(imu_file, measurements)) {
        std::cout << "load data failed!" << std::endl;
        return -1;
    }
    if(!load_ground_truth(ground_truth_file, ground_truths)) {
        std::cout << "load data failed!" << std::endl;
        return -1;
    }
    std::cout << "measurement length: " << measurements.size() << std::endl;
    std::cout << "ground truth length: " << ground_truths.size() << std::endl;

    // estimate
    InsFilter filter(false);
    GroundTruthPackage init_state;
    init_state.timestamp_ = 91620.0;
    init_state.pos_ = Eigen::Vector3d(23.1373950708, 113.3713651222, 2.175);
    init_state.vel_ = Eigen::Vector3d(0, 0, 0);
    init_state.att_ = Eigen::Vector3d(0.0107951084511778, -2.14251290749072, -75.7498049314083);
    filter.Initialize(init_state);
    std::vector<GroundTruthPackage> results;
    for(auto& measurement : measurements) {
        if(measurement.timestamp_ <= init_state.timestamp_) {
            continue;
        }
        filter.Update(measurement);
        GroundTruthPackage result = filter.GetState();
        results.push_back(result);
    }
    save(result_file, results);

    // calculate error
    Eigen::Vector3d avg_pos_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_vel_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_att_error = Eigen::Vector3d::Zero();
    auto result_iter = results.begin();
    auto ground_truth_iter = ground_truths.begin();
    int count = 0;
    while(result_iter != results.end() && ground_truth_iter != ground_truths.end()) {
        if(result_iter->timestamp_ == ground_truth_iter->timestamp_) {
            Eigen::Vector3d pos_error = (result_iter->pos_ - ground_truth_iter->pos_).cwiseAbs();
            Eigen::Vector3d vel_error = (result_iter->vel_ - ground_truth_iter->vel_).cwiseAbs();
            Eigen::Vector3d att_error = (result_iter->att_ - ground_truth_iter->att_).cwiseAbs();
            avg_pos_error += pos_error;
            avg_vel_error += vel_error;
            avg_att_error += att_error;
            count++;
            result_iter++;
            ground_truth_iter++;
        } else if(result_iter->timestamp_ < ground_truth_iter->timestamp_) {
            result_iter++;
        } else {
            ground_truth_iter++;
        }
    }
    avg_pos_error /= double(count);
    avg_vel_error /= double(count);
    avg_att_error /= double(count);
    std::cout << "matched result count " << count << std::endl;
    std::cout << "average position error " << std::scientific << avg_pos_error.transpose() << std::endl;
    std::cout << "average velocity error " << std::scientific << avg_vel_error.transpose() << std::endl;
    std::cout << "average attitude error " << std::scientific << avg_att_error.transpose() << std::endl;

    return 0;
}