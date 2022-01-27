#include "eskf_odometry_ros_alg.h"

#include <rot_fc.h>

EskfOdomAlgorithm::EskfOdomAlgorithm(void)
{
    pthread_mutex_init(&this->access_,NULL);
}

EskfOdomAlgorithm::~EskfOdomAlgorithm(void)
{
    pthread_mutex_destroy(&this->access_);
}

void EskfOdomAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
    this->lock();

    // save the current configuration
    this->config_=new_cfg;

    this->unlock();
}

void EskfOdomAlgorithm::set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu, const Sensor::position_params& position, const Sensor::orientation_params& orientation, const Sensor::gravity_params& gravity, const Sensor::magnetometer_params& magnetometer)
{
    this->filter_.set_init_params(f_params, x0, dx0, imu, position, orientation, gravity, magnetometer );
}

void EskfOdomAlgorithm::print_ini_params(void)
{
    this->filter_.print_ini_params();
}

int EskfOdomAlgorithm::set_imu_reading(const float& t_msg, const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::MatrixXf& Ra, const Eigen::MatrixXf& Rw)
{
    int result;
    result = this->filter_.set_imu_reading(t_msg, a, w, Ra, Rw); 
    return result;
}

int EskfOdomAlgorithm::set_position_reading(const float& t_msg, const Eigen::Vector3f& msg, const Eigen::Matrix3f& R)
{
    int result;
    result = this->filter_.set_position_reading(t_msg, msg, R);
    return result;
}

int EskfOdomAlgorithm::set_magnetometer_reading(const float& t_msg, const Eigen::VectorXf& msg, const Eigen::MatrixXf& R)
{
    int result;
    result = this->filter_.set_magnetometer_reading(t_msg, msg, R);  
    return result;
}

Eigen::Quaternionf rotVecToQuat(const Eigen::Vector3f& in) {
    float angle = in.norm();
    Eigen::Vector3f axis = (angle == 0) ? Eigen::Vector3f(1, 0, 0) : in.normalized();
    return Eigen::Quaternionf(AngleAxisf(angle, axis));
}

Eigen::Vector3f quatToRotVec(const Eigen::Quaternionf& q) {
    Eigen::AngleAxisf angAx(q);
    return angAx.angle() * angAx.axis();
}


int EskfOdomAlgorithm::set_orientation_reading(const float& t, const Eigen::Quaternionf& q_gb_meas, const Eigen::Matrix3f& theta_covariance, const Eigen::Quaternionf& nwu_q_imu)
{
    int result;
    result = this->filter_.set_orientation_reading(t, q_gb_meas, theta_covariance);
    return result;
}

void EskfOdomAlgorithm::set_linvel_reading(const float& t, const Eigen::Vector3f& val)
{
    // this->filter_.set_linvel_reading(t,val);
}

void EskfOdomAlgorithm::set_range_reading(const float& t, const Eigen::VectorXf& val)
{
    // this->filter_.set_range_reading(t,val);
}

void EskfOdomAlgorithm::set_px4_reading(const float& t, const Eigen::VectorXf& data)
{
    // this->filter_.set_px4_reading(t,data);
}

float EskfOdomAlgorithm::get_proc_time(void)
{
    // return this->filter_.get_proc_time();
}
bool EskfOdomAlgorithm::update(Eigen::VectorXf& state, Eigen::Vector3f& ang_vel, const bool& flying, const float& gnd_dist)
{
    // return this->filter_.update(state,ang_vel, flying, gnd_dist);
}
void EskfOdomAlgorithm::update(Eigen::VectorXf& state, Eigen::VectorXf& covPtrue, Eigen::VectorXf& dxstate)
{
    this->filter_.update(state, covPtrue, dxstate);
}