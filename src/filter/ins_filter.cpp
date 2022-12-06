//
// Created by lihao on 2022/10/5.
//

#include "ins_filter.h"


Eigen::Vector3d Quaternion2EulerAngles(const Eigen::Quaterniond& quaternion) {
    Eigen::Matrix3d R(quaternion);
    Eigen::Vector3d eulerAngles;
    eulerAngles[0] = atan2(R(2, 1), R(2, 2));
    eulerAngles[1] = atan2(-R(2, 0), sqrt(1.0 - R(2, 0) * R(2, 0)));
    eulerAngles[2] = atan2(R(1, 0),R(0, 0));
    return eulerAngles;
}

Eigen::Quaterniond EulerAngles2Quaternion(const Eigen::Vector3d& eulerAngles) {
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
    return quaternion;
}

InsFilter::InsFilter(bool use_extrapolation)
    : use_extrapolation_(use_extrapolation)
{
    is_initialized_ = false;
    last_timestamp_ = 0;
    timestamp_ = 0;
    delta_t_ = 0;

    last_gyro_ = Eigen::Vector3d::Zero();
    last_acc_ = Eigen::Vector3d::Zero();
    avg_vel_ = Eigen::Vector3d::Zero();
    avg_pos_ = Eigen::Vector3d::Zero();
}

void InsFilter::Initialize(GroundTruthPackage& state) {
    Initialize(state.timestamp_, state.pos_, state.vel_, state.att_);
}

void InsFilter::Initialize(double timestamp, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& att) {
    last_timestamp_ = timestamp;
    last_pos_ = pos;
    last_vel_ = vel;
    last_att_ = EulerAngles2Quaternion(degree2rad(att));
}

void InsFilter::Update(MeasurementPackage measurement) {
    Update(measurement.timestamp_, measurement.gyro_, measurement.acc_);
}

void InsFilter::Update(double timestamp, Eigen::Vector3d& gyro, Eigen::Vector3d& acc) {
    timestamp_ = timestamp;
    gyro_ = gyro;
    acc_ = acc;
    delta_t_ = timestamp_ - last_timestamp_;

    // 速度和位置外插
    if(use_extrapolation_) {
        avg_vel_ = last_vel_ + 0.5 * avg_vel_ * delta_t_;
        avg_pos_ = last_pos_ + 0.5 * avg_pos_ * delta_t_;
    } else {
        avg_vel_ = last_vel_;
        avg_pos_ = last_pos_;
    }

    // 估计
    EstimateAttitude();
    EstimateVelocity();
    EstimatePosition();

    // 更新
    avg_vel_ = (vel_ - last_vel_) / fmax(delta_t_, EPSILON);
    avg_pos_ = (pos_ - last_pos_) / fmax(delta_t_, EPSILON);
    last_timestamp_ = timestamp_;
    last_gyro_ = gyro_;
    last_acc_ = acc_;
    last_att_ = att_;
    last_vel_ = vel_;
    last_pos_ = pos_;
}

GroundTruthPackage InsFilter::GetState() {
    GroundTruthPackage state;
    state.timestamp_ = last_timestamp_;
    state.pos_ = last_pos_;
    state.vel_ = last_vel_;
    state.att_ = rad2degree(Quaternion2EulerAngles(last_att_));
    return state;
}

Eigen::Vector3d InsFilter::GetPosition() {
    return last_pos_;
}

Eigen::Vector3d InsFilter::GetVelocity() {
    return last_vel_;
}

Eigen::Vector3d InsFilter::GetAttitude() {
    return rad2degree(Quaternion2EulerAngles(last_att_));
}

void InsFilter::EstimateAttitude() {
    double avg_lat = degree2rad(avg_pos_[0]);
    double avg_height = avg_pos_[2];

    // 计算相对选择向量
    Eigen::Vector3d r_b_k = gyro_ + last_gyro_.cross(gyro_) / 12.0;
    Eigen::Quaterniond q_b_k(Eigen::AngleAxisd(r_b_k.norm(), r_b_k.normalized()));

    double Rm = EARTH_SEMI_MAJOR_AXIS * (1.0 - pow(EARTH_ECCENTRICITY, 2)) /
            pow(1.0 - pow(EARTH_ECCENTRICITY * sin(avg_lat), 2.0), 1.5);
    double Rn = EARTH_SEMI_MAJOR_AXIS / sqrt(1.0 - pow(EARTH_ECCENTRICITY * sin(avg_lat), 2.0));

    omega_en_ = Eigen::Vector3d(
            avg_vel_[1] / (Rn + avg_height),
            -avg_vel_[0] / (Rm + avg_height),
            -avg_vel_[1] * tan(avg_lat) / (Rn + avg_height)
            );
    omega_ie_ = Eigen::Vector3d (
            EARTH_ROTATION_VELOCITY * cos(avg_lat),
            0.0,
            -EARTH_ROTATION_VELOCITY * sin(avg_lat)
            );
    zeta_ = (omega_en_ + omega_ie_) * delta_t_;
    Eigen::Quaterniond q_n_k(Eigen::AngleAxisd(-zeta_.norm(), zeta_.normalized()));
    Eigen::Quaterniond q_n_b = q_n_k * last_att_ * q_b_k;
    att_ = q_n_b.normalized();
}

void InsFilter::EstimateVelocity() {
    double avg_lat = degree2rad(avg_pos_[0]);
    double avg_height = avg_pos_[2];

    // 计算重力加速度的系数
    double grav_coeff[6] = {9.7803267715, 0.0052790414, 0.0000232718,
                            -0.000003087691089, 0.000000004397731, 0.000000000000721};
    double sin_lat = sin(avg_lat);
    double sin_lat2 = pow(sin_lat, 2.0);
    double sin_lat4 = pow(sin_lat2, 2.0);
    double g1 = grav_coeff[0] * (1.0 + grav_coeff[1] * sin_lat2 + grav_coeff[2] * sin_lat4);
    double g2 = (grav_coeff[3] + grav_coeff[4] * sin_lat2) * avg_height;
    double g3 = grav_coeff[5] * pow(avg_height, 2.0);
    Eigen::Vector3d g(0, 0, g1 + g2 + g3);
    Eigen::Vector3d delta_v_g = (g - (2 * omega_ie_ + omega_en_).cross(avg_vel_)) * delta_t_;

    Eigen::Vector3d v_f = acc_ + 0.5 * gyro_.cross(acc_) +
            (last_gyro_.cross(acc_) + last_acc_.cross(gyro_)) / 12.0;
    Eigen::Matrix3d R_n_b = last_att_.matrix();
    Eigen::Matrix3d zeta_anti;
    zeta_anti << 0, -zeta_[2], zeta_[1],
            zeta_[2], 0, -zeta_[0],
            -zeta_[1], zeta_[0], 0;
    Eigen::Vector3d delta_v_f = (Eigen::Matrix3d::Identity() - 0.5 * zeta_anti) * R_n_b * v_f;
    vel_ = last_vel_ + delta_v_g + delta_v_f;
}

void InsFilter::EstimatePosition() {
    double avg_lat = degree2rad(avg_pos_[0]);
    double last_lat = degree2rad(last_pos_[0]);
    double last_lon = degree2rad(last_pos_[1]);
    double last_height = last_pos_[2];
    avg_vel_ = 0.5 * (last_vel_ + vel_);

    // 高度
    double height = last_height - avg_vel_[2] * delta_t_;
    
    // 维度
    double avg_height = 0.5 * (last_height + height);
    double Rm = EARTH_SEMI_MAJOR_AXIS * (1.0 - pow(EARTH_ECCENTRICITY, 2)) /
                pow(1.0 - pow(EARTH_ECCENTRICITY * sin(avg_lat), 2.0), 1.5);
    double lat = last_lat + avg_vel_[0] / (Rm + avg_height) * delta_t_;
    
    // 经度
    avg_lat = 0.5 * (last_lat + lat);
    double Rn = EARTH_SEMI_MAJOR_AXIS / sqrt(1.0 - pow(EARTH_ECCENTRICITY * sin(avg_lat), 2.0));
    double lon = last_lon + avg_vel_[1] / ((Rn + avg_height) * cos(avg_lat)) * delta_t_;
    pos_ = Eigen::Vector3d(rad2degree(lat), rad2degree(lon), height);
}