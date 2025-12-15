#ifndef SE3_MID_LEVEL_CONTROLLER_H
#define SE3_MID_LEVEL_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> // Matris yayını için
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <algorithm> // std::max, std::min için

namespace MidLevelNS {
    // PID hesaplama fonksiyonu (Actual - Desired yapısına göre çalışır)
    double computePID(double des_pos, double a_pos, double des_vel, double a_vel, double& integral,
                      double kp, double ki, double kd, double dt, double integral_min, double integral_max);
}

class MidLevelController {
public:
    MidLevelController();
    void spin();

private:
    // ROS Handle
    ros::NodeHandle m_nh;

    // Subscribers
    ros::Subscriber m_desired_position_sub;
    ros::Subscriber m_current_position_sub;
    ros::Subscriber m_desired_velocity_sub;
    ros::Subscriber m_current_velocity_sub;
    ros::Subscriber m_desired_acceleration_sub;
    ros::Subscriber m_desired_psi_sub;
    ros::Subscriber m_disturbance_sub;
    
    // Türev verileri için subscriberlar
    ros::Subscriber m_desired_jerk_sub;
    ros::Subscriber m_desired_d_psi_sub;

    // Publishers
    ros::Publisher m_desired_thrust_pub;
    ros::Publisher m_desired_rotation_pub; // Rotation Matrix (3x3)
    ros::Publisher m_desired_omega_pub;    // Angular Velocity (wx, wy, wz)
    ros::Publisher m_thrust_components_pub; // Debug için (Opsiyonel)

    // Target and Current States
    double m_des_x, m_des_y, m_des_z, m_des_psi;
    double m_current_x, m_current_y, m_current_z;
    
    double m_desired_velocity_x, m_desired_velocity_y, m_desired_velocity_z;
    double m_current_velocity_x, m_current_velocity_y, m_current_velocity_z;
    
    double m_desired_acceleration_x, m_desired_acceleration_y, m_desired_acceleration_z;
    double m_disturbance_x, m_disturbance_y, m_disturbance_z;
    
    // Jerk ve Psi Türevi Değişkenleri
    double m_ref_jerk_x, m_ref_jerk_y, m_ref_jerk_z;
    double m_ref_d_psi; // Veri gelmezse 0.0

    // Thrust PID Gains
    double m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x;
    double m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y;
    double m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z;
    
    // Limits & System Constants
    double m_min_thrust, m_max_thrust;
    double m_integral_min, m_integral_max;
    double m_dt;
    double m_mass;

    // PID State Variables (Integrals)
    double m_prev_error_thrust_x, m_integral_thrust_x;
    double m_prev_error_thrust_y, m_integral_thrust_y;
    double m_prev_error_thrust_z, m_integral_thrust_z;
    
    // Geometrik Kontrol Durum Değişkenleri
    Eigen::Vector3d m_F_des;    // Hesaplanan İstenen Kuvvet Vektörü
    Eigen::Matrix3d m_prev_R_d; // Önceki adımın rotasyon matrisi (Singularity için)

    // --- Fonksiyonlar ---
    
    // Thrust hesaplar, m_F_des'i günceller ve thrust normunu döner
    double computeThrust();
    
    // Rotasyon matrisini ve açısal hızı (omega) hesaplar ve yayınlar
    void computeAndPublishRotation(); 
    
    // Thrust limiti uygular
    double applyThrustSaturation(double thrust, double min_thrust, double max_thrust);

    // Birim vektör türevi için yardımcı fonksiyon: d(u/|u|)
    Eigen::Vector3d calculateUnitVectorDerivative(const Eigen::Vector3d& v, const Eigen::Vector3d& dv, double norm_v);

    // Callbacks
    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredPsiCallback(const std_msgs::Float64::ConstPtr& msg);
    void disturbanceCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredJerkCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void desiredDPsiCallback(const std_msgs::Float64::ConstPtr& msg);
};

#endif // SE3_MID_LEVEL_CONTROLLER_H