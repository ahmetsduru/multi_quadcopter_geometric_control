#include "../include/multi_quadcopter_geometric_control/SE3_low_level_cont.h"
#include <vector>

namespace LowLevelNS {
    Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
        Eigen::Vector3d v;
        v << S(2, 1), S(0, 2), S(1, 0);
        return v;
    }
} 

LowLevelController::LowLevelController()
    : kp_torque1(0), ki_torque1(0), kd_torque1(0),
      kp_torque2(0), ki_torque2(0), kd_torque2(0),
      kp_torque3(0), ki_torque3(0), kd_torque3(0) {

    // Parametreleri Yükle
    nh.getParam("low_level_controller/kp_torque1", kp_torque1);
    nh.getParam("low_level_controller/ki_torque1", ki_torque1); 
    nh.getParam("low_level_controller/kd_torque1", kd_torque1);
    
    nh.getParam("low_level_controller/kp_torque2", kp_torque2);
    nh.getParam("low_level_controller/ki_torque2", ki_torque2); 
    nh.getParam("low_level_controller/kd_torque2", kd_torque2);
    
    nh.getParam("low_level_controller/kp_torque3", kp_torque3);
    nh.getParam("low_level_controller/ki_torque3", ki_torque3); 
    nh.getParam("low_level_controller/kd_torque3", kd_torque3);
    
    nh.getParam("low_level_controller/dt", dt);
    nh.getParam("low_level_controller/integral_min", integral_min);
    nh.getParam("low_level_controller/integral_max", integral_max);

    std::vector<double> I_vec;
    nh.getParam("state_derivative_solver_node/I", I_vec);
    m_J = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(I_vec.data());

    // Subscriberlar
    ref_rot_sub = nh.subscribe("reference_rotation_matrix", 10, &LowLevelController::referenceRotationCallback, this);
    act_rot_sub = nh.subscribe("actual_rotation_matrix", 10, &LowLevelController::currentRotationCallback, this);
    ref_omega_sub = nh.subscribe("reference_angular_velocity", 10, &LowLevelController::referenceOmegaCallback, this);
    act_omega_sub = nh.subscribe("actual_angular_velocity", 10, &LowLevelController::currentOmegaCallback, this);

    torque_pub = nh.advertise<geometry_msgs::Vector3>("reference_torques", 10);
    
    // Başlangıç değerleri
    m_R_d.setIdentity();
    m_R_a.setIdentity();
    m_omega_d.setZero();
    m_omega_a.setZero();
    
    m_integral_error.setZero();
}

void LowLevelController::spin() {
    ros::Rate rate(1/dt); 
    while (ros::ok()) {
        ros::spinOnce();
        computeAndPublishTorques();
        rate.sleep();
    }
}

void LowLevelController::computeAndPublishTorques() {
    // 1. R_err Hesabı
    Eigen::Matrix3d R_err_matrix = m_R_d.transpose() * m_R_a - m_R_a.transpose() * m_R_d;
    Eigen::Vector3d R_err = 0.5 * LowLevelNS::vee(R_err_matrix);

    // 2. Omega_err Hesabı
    Eigen::Vector3d omega_err = m_omega_a - (m_R_a.transpose() * m_R_d * m_omega_d);

    // 3. İntegral Hata Güncellemesi
    m_integral_error += R_err * dt;

    // Anti-Windup
    for (int i = 0; i < 3; ++i) {
        if (m_integral_error(i) > integral_max) m_integral_error(i) = integral_max;
        else if (m_integral_error(i) < integral_min) m_integral_error(i) = integral_min;
    }

    // 4. Kazanç Vektörleri
    Eigen::Vector3d Kp, Ki, Kd;
    Kp << kp_torque1, kp_torque2, kp_torque3;
    Ki << ki_torque1, ki_torque2, ki_torque3;
    Kd << kd_torque1, kd_torque2, kd_torque3;

    // 5. Tork Hesabı
    Eigen::Vector3d Gyro = m_omega_a.cross(m_J * m_omega_a);
    
    Eigen::Vector3d torques = -Kp.cwiseProduct(R_err) - Ki.cwiseProduct(m_integral_error) - Kd.cwiseProduct(omega_err) + Gyro;

    // 6. Publish
    geometry_msgs::Vector3 torque_msg;
    torque_msg.x = torques.x();
    torque_msg.y = torques.y();
    torque_msg.z = torques.z();
    torque_pub.publish(torque_msg);
}

// --- Callback Fonksiyonları ---

void LowLevelController::referenceRotationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if(msg->data.size() == 9) m_R_d = arrayToMatrix(msg->data);
}

void LowLevelController::currentRotationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if(msg->data.size() == 9) m_R_a = arrayToMatrix(msg->data);
}

void LowLevelController::referenceOmegaCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_omega_d << msg->x, msg->y, msg->z;
}

void LowLevelController::currentOmegaCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_omega_a << msg->x, msg->y, msg->z;
}

// Eigen::Map optimizasyonu ile matris dönüşümü
Eigen::Matrix3d LowLevelController::arrayToMatrix(const std::vector<double>& data) {
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(data.data());
}