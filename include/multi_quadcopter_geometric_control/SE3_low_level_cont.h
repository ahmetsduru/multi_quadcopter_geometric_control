#ifndef SE3_LOWLEVELCONTROLLER_H
#define SE3_LOWLEVELCONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h> // Matris verileri için
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <algorithm> // std::max, std::min için

namespace LowLevelNS {
    Eigen::Vector3d vee(const Eigen::Matrix3d& S);
}

class LowLevelController {
public:
    LowLevelController();
    void spin();

private:
    ros::NodeHandle nh;

    // --- Subscriberlar ---
    ros::Subscriber ref_rot_sub;   // Desired Rotation Matrix (R_d)
    ros::Subscriber act_rot_sub;   // Actual Rotation Matrix (R_a)
    ros::Subscriber ref_omega_sub; // Desired Angular Velocity (Omega_d)
    ros::Subscriber act_omega_sub; // Actual Angular Velocity (Omega_a)

    // Publisher
    ros::Publisher torque_pub;

    // --- Durum Değişkenleri (Eigen Nesneleri) ---
    Eigen::Matrix3d m_R_d;      // İstenen Rotasyon
    Eigen::Matrix3d m_R_a;      // Gerçek Rotasyon
    Eigen::Vector3d m_omega_d;  // İstenen Açısal Hız
    Eigen::Vector3d m_omega_a;  // Gerçek Açısal Hız
    
    Eigen::Matrix3d m_J;              // Eylemsizlik Matrisi (Inertia)
    Eigen::Vector3d m_integral_error; // İntegral Hatası Birikimi

    // --- Kazançlar ve Parametreler ---
    // 1 -> X ekseni, 2 -> Y ekseni, 3 -> Z ekseni
    double kp_torque1, ki_torque1, kd_torque1;
    double kp_torque2, ki_torque2, kd_torque2;
    double kp_torque3, ki_torque3, kd_torque3;
    
    double dt;
    double integral_min, integral_max;

    // --- Fonksiyonlar ---
    void computeAndPublishTorques();

    // Callback Fonksiyonları
    void referenceRotationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void currentRotationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void referenceOmegaCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentOmegaCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    // Yardımcı Fonksiyon
    Eigen::Matrix3d arrayToMatrix(const std::vector<double>& data);
};

#endif // SE3_LOWLEVELCONTROLLER_H