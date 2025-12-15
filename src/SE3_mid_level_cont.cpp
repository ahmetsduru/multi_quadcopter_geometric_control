#include "../include/multi_quadcopter_geometric_control/SE3_mid_level_cont.h"

namespace MidLevelNS {

// PID Fonksiyonu: Hata = (Actual - Desired) mantığıyla integral alır
double computePID(double des_pos, double a_pos, double des_vel, double a_vel, double& integral,
                  double kp, double ki, double kd, double dt, double integral_min, double integral_max) {   
    double pos_error = a_pos - des_pos;
    double vel_error = a_vel - des_vel;
    
    integral += pos_error * dt;
    
    // Anti-windup
    if (integral > integral_max) integral = integral_max;
    else if (integral < integral_min) integral = integral_min;

    return kp * pos_error + ki * integral + kd * vel_error;
}
}

MidLevelController::MidLevelController() 
    : m_des_x(0.0), m_des_y(0.0), m_des_z(0.0), m_des_psi(0.0),
      m_current_x(0.0), m_current_y(0.0), m_current_z(0.0),
      m_prev_error_thrust_x(0.0), m_integral_thrust_x(0.0),
      m_prev_error_thrust_y(0.0), m_integral_thrust_y(0.0),
      m_prev_error_thrust_z(0.0), m_integral_thrust_z(0.0),
      m_desired_acceleration_x(0.0), m_desired_acceleration_y(0.0), m_desired_acceleration_z(0.0), 
      m_disturbance_x(0.0), m_disturbance_y(0.0), m_disturbance_z(0.0),
      m_ref_jerk_x(0.0), m_ref_jerk_y(0.0), m_ref_jerk_z(0.0),
      m_ref_d_psi(0.0)
      {

    m_nh.getParam("mid_level_controller/kp_thrust_x", m_kp_thrust_x);
    m_nh.getParam("mid_level_controller/ki_thrust_x", m_ki_thrust_x);
    m_nh.getParam("mid_level_controller/kd_thrust_x", m_kd_thrust_x);
    m_nh.getParam("mid_level_controller/kp_thrust_y", m_kp_thrust_y);
    m_nh.getParam("mid_level_controller/ki_thrust_y", m_ki_thrust_y);
    m_nh.getParam("mid_level_controller/kd_thrust_y", m_kd_thrust_y);
    m_nh.getParam("mid_level_controller/kp_thrust_z", m_kp_thrust_z);
    m_nh.getParam("mid_level_controller/ki_thrust_z", m_ki_thrust_z);
    m_nh.getParam("mid_level_controller/kd_thrust_z", m_kd_thrust_z);
    m_nh.getParam("mid_level_controller/dt", m_dt);
    m_nh.getParam("mid_level_controller/min_thrust", m_min_thrust);
    m_nh.getParam("mid_level_controller/max_thrust", m_max_thrust);
    m_nh.getParam("mid_level_controller/integral_min", m_integral_min);
    m_nh.getParam("mid_level_controller/integral_max", m_integral_max);
    m_nh.getParam("state_derivative_solver_node/mass", m_mass);

    // Subscriberlar
    m_desired_position_sub = m_nh.subscribe("reference_position", 10, &MidLevelController::positionCallback, this);
    m_current_position_sub = m_nh.subscribe("actual_position", 10, &MidLevelController::currentPositionCallback, this);
    m_desired_velocity_sub = m_nh.subscribe("reference_velocity", 10, &MidLevelController::desiredVelocityCallback, this);
    m_current_velocity_sub = m_nh.subscribe("actual_velocity", 10, &MidLevelController::currentVelocityCallback, this);
    m_desired_acceleration_sub = m_nh.subscribe("reference_acceleration", 10, &MidLevelController::desiredAccelerationCallback, this);
    m_desired_psi_sub = m_nh.subscribe("reference_psi", 10, &MidLevelController::desiredPsiCallback, this);
    m_disturbance_sub = m_nh.subscribe("disturbance_force", 10, &MidLevelController::disturbanceCallback, this);
    m_desired_jerk_sub = m_nh.subscribe("reference_jerk", 10, &MidLevelController::desiredJerkCallback, this);
    m_desired_d_psi_sub = m_nh.subscribe("reference_d_psi", 10, &MidLevelController::desiredDPsiCallback, this);

    // Publisherlar
    m_desired_thrust_pub = m_nh.advertise<std_msgs::Float64>("reference_thrust", 10);
    m_desired_rotation_pub = m_nh.advertise<std_msgs::Float64MultiArray>("reference_rotation_matrix", 10);
    m_desired_omega_pub = m_nh.advertise<geometry_msgs::Vector3>("reference_angular_velocity", 10);
    
    // Durum Değişkenlerini Başlat
    m_F_des << 0.0, 0.0, m_mass * 9.81;
    m_prev_R_d.setIdentity(); 
}

void MidLevelController::spin() {
    ros::Rate rate(1 / m_dt); 
    while (ros::ok()) {
        ros::spinOnce();

        // 1. Thrust Vektörünü ve Büyüklüğünü Hesapla
        double thrust_norm = computeThrust();
        
        // 2. Rotasyon Matrisini ve Açısal Hızı (Omega) Hesapla ve Yayınla
        computeAndPublishRotation(); 
        
        // 3. Thrust Büyüklüğünü Yayınla
        std_msgs::Float64 thrust_msg;
        thrust_msg.data = thrust_norm;
        m_desired_thrust_pub.publish(thrust_msg);

        rate.sleep();
    }
}

double MidLevelController::computeThrust() {
    // --- ADIM 1: İstenen Kuvvet Vektörünün (F_des) Hesaplanması ---
    
    // PID Fonksiyonu (Actual - Desired) döndürdüğü için işareti negatiftir.
    double pid_x = MidLevelNS::computePID(m_des_x, m_current_x, m_desired_velocity_x, m_current_velocity_x, m_integral_thrust_x, m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x, m_dt, m_integral_min, m_integral_max);
    double pid_y = MidLevelNS::computePID(m_des_y, m_current_y, m_desired_velocity_y, m_current_velocity_y, m_integral_thrust_y, m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y, m_dt, m_integral_min, m_integral_max);
    double pid_z = MidLevelNS::computePID(m_des_z, m_current_z, m_desired_velocity_z, m_current_velocity_z, m_integral_thrust_z, m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z, m_dt, m_integral_min, m_integral_max);

    // F_des = -PID_Out + m*a_ref + m*g - disturbance
    //double f_x = -pid_x + m_mass * m_desired_acceleration_x - m_disturbance_x;
    //double f_y = -pid_y + m_mass * m_desired_acceleration_y - m_disturbance_y;
    //double f_z = -pid_z + m_mass * m_desired_acceleration_z + m_mass * 9.81 - m_disturbance_z;

    double f_x = -pid_x + m_mass * m_desired_acceleration_x;
    double f_y = -pid_y + m_mass * m_desired_acceleration_y;
    double f_z = -pid_z + m_mass * m_desired_acceleration_z + m_mass * 9.81;

    // Vektörü kaydet (Matris hesabında kullanılacak)
    m_F_des << f_x, f_y, f_z;

    // Toplam itki büyüklüğü
    double total_thrust = m_F_des.norm();
    
    return applyThrustSaturation(total_thrust, m_min_thrust, m_max_thrust);
}

void MidLevelController::computeAndPublishRotation() {
    // --- ADIM 2: İstenen Yönelimin (R_d) Hesaplanması ---
    Eigen::Vector3d F_des = m_F_des;
    double norm_thrust = F_des.norm();

    // A) b3d (Z Ekseni)
    Eigen::Vector3d b3d; 
    if (norm_thrust < 1e-6) {
        b3d = m_prev_R_d.col(2); // Tekillik durumu
    } else {
        b3d = F_des.normalized();
    }

    // B) b1c (Heading Vektörü)
    Eigen::Vector3d b1c(cos(m_des_psi), sin(m_des_psi), 0.0);

    // C) b2d (Y Ekseni)
    Eigen::Vector3d C_vec = b3d.cross(b1c);
    double norm_C = C_vec.norm();
    Eigen::Vector3d b2d;
    
    if (norm_C < 1e-6) {
        b2d = m_prev_R_d.col(1); // Tekillik durumu
    } else {
        b2d = C_vec / norm_C;
    }

    // D) b1d (X Ekseni)
    Eigen::Vector3d b1d = b2d.cross(b3d);

    // E) Rotasyon Matrisi (R_d)
    Eigen::Matrix3d R_d;
    R_d.col(0) = b1d;
    R_d.col(1) = b2d;
    R_d.col(2) = b3d;

    m_prev_R_d = R_d; // Bir sonraki adım için sakla

    // --- ADIM 4: Açısal Hız Komutu (Omega_des) için Türevler ---
    
    // A) Kuvvet Türevi (dF_des)
    // dF = -Kp * error_v + m * jerk (Kd ihmal, d(error_p) = error_v)
    double err_vx = m_current_velocity_x - m_desired_velocity_x;
    double err_vy = m_current_velocity_y - m_desired_velocity_y;
    double err_vz = m_current_velocity_z - m_desired_velocity_z;

    Eigen::Vector3d dF_des;
    dF_des.x() = -m_kp_thrust_x * err_vx + m_mass * m_ref_jerk_x;
    dF_des.y() = -m_kp_thrust_y * err_vy + m_mass * m_ref_jerk_y;
    dF_des.z() = -m_kp_thrust_z * err_vz + m_mass * m_ref_jerk_z;

    // B) db3d Türevi
    Eigen::Vector3d db3d = calculateUnitVectorDerivative(F_des, dF_des, norm_thrust);

    // db1c Türevi (Zincir kuralı: d/dt [cos(psi), sin(psi), 0])
    Eigen::Vector3d db1c(-sin(m_des_psi) * m_ref_d_psi, cos(m_des_psi) * m_ref_d_psi, 0.0);

    // C) db2d Türevi
    // dC = db3d x b1c + b3d x db1c
    Eigen::Vector3d dC_vec = db3d.cross(b1c) + b3d.cross(db1c);
    Eigen::Vector3d db2d = calculateUnitVectorDerivative(C_vec, dC_vec, norm_C);

    // D) db1d Türevi
    Eigen::Vector3d db1d = db2d.cross(b3d) + b2d.cross(db3d);

    // E) R_d Türevi (dR_d)
    Eigen::Matrix3d dR_d;
    dR_d.col(0) = db1d;
    dR_d.col(1) = db2d;
    dR_d.col(2) = db3d;

    // Omega Hesabı: S(w) = R_d^T * dR_d
    Eigen::Matrix3d S_omega = R_d.transpose() * dR_d;

    // Vee Map (Matristen Vektöre)
    geometry_msgs::Vector3 des_omega_msg;
    des_omega_msg.x = S_omega(2, 1);
    des_omega_msg.y = S_omega(0, 2);
    des_omega_msg.z = S_omega(1, 0);

    // --- YAYINLAMA (PUBLISH) ---

    // 1. Rotasyon Matrisi Yayınla (Row-Major array)
    std_msgs::Float64MultiArray R_msg;
    R_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    R_msg.layout.dim[0].label = "row_major";
    R_msg.layout.dim[0].size = 9;
    R_msg.layout.dim[0].stride = 9;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_msg.data.push_back(R_d(i, j));
        }
    }
    m_desired_rotation_pub.publish(R_msg);

    // 2. Açısal Hız Yayınla
    m_desired_omega_pub.publish(des_omega_msg);
}

Eigen::Vector3d MidLevelController::calculateUnitVectorDerivative(const Eigen::Vector3d& v, const Eigen::Vector3d& dv, double norm_v) {
    // Formül: d(v/|v|) = (dv*|v| - v*(v.dv)/|v|) / |v|^2
    if (norm_v < 1e-6) return Eigen::Vector3d::Zero();

    double v_dot_dv = v.dot(dv);
    Eigen::Vector3d term1 = dv * norm_v;
    Eigen::Vector3d term2 = v * (v_dot_dv / norm_v);
    
    return (term1 - term2) / (norm_v * norm_v);
}

double MidLevelController::applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    return std::max(min_thrust, std::min(thrust, max_thrust));
}

// --- Callbacks ---

void MidLevelController::positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_des_x = msg->x; m_des_y = msg->y; m_des_z = msg->z;
}
void MidLevelController::currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_x = msg->x; m_current_y = msg->y; m_current_z = msg->z;
}
void MidLevelController::desiredVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_velocity_x = msg->x; m_desired_velocity_y = msg->y; m_desired_velocity_z = msg->z;
}
void MidLevelController::currentVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_velocity_x = msg->x; m_current_velocity_y = msg->y; m_current_velocity_z = msg->z;
}
void MidLevelController::desiredAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_acceleration_x = msg->x; m_desired_acceleration_y = msg->y; m_desired_acceleration_z = msg->z;
}
void MidLevelController::desiredPsiCallback(const std_msgs::Float64::ConstPtr& msg) {
    m_des_psi = msg->data;
}
void MidLevelController::disturbanceCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_disturbance_x = msg->x; m_disturbance_y = msg->y; m_disturbance_z = msg->z;
}
void MidLevelController::desiredJerkCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_ref_jerk_x = msg->x;
    m_ref_jerk_y = msg->y;
    m_ref_jerk_z = msg->z;
}
void MidLevelController::desiredDPsiCallback(const std_msgs::Float64::ConstPtr& msg) {
    m_ref_d_psi = msg->data;
}