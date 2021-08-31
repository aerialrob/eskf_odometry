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

// void EskfOdomAlgorithm::set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu,const Sensor::pose_params& pose,const Sensor::pose_params& pose2,const Sensor::position_params& position,const Sensor::orientation_params& orientation,const Sensor::linvel_params& linvel,const Sensor::range_params& range,const Sensor::px4_params& px4,const Sensor::flow2d_params& flow2d)
// {
//     // this->filter_.set_init_params(f_params,x0,dx0,imu,pose,pose2,position,orientation,linvel,range,px4,flow2d);
// }

void EskfOdomAlgorithm::set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu,const Sensor::pose_params& pose,const Sensor::pose_params& pose2,const Sensor::position_params& position,const Sensor::orientation_params& orientation,const Sensor::linvel_params& linvel)
{
    this->filter_.set_init_params(f_params,x0,dx0,imu,pose,pose2,position,orientation,linvel);
}

void EskfOdomAlgorithm::print_ini_params(void)
{
    this->filter_.print_ini_params();
}

// Sensor::imu_params EskfOdomAlgorithm::get_imu_params(void)
// {
//     // return this->filter_.get_imu_params();
// }

// void EskfOdomAlgorithm::set_imu_params(Sensor::imu_params& imu)
// {
//     // this->filter_.set_imu_params(imu);
// }

void EskfOdomAlgorithm::propagate_imu(const float& dt, const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::Quaternionf& nwu_q_imu)
{
//      In case Imu has other orientation

//    std::string imu_frame_ori = "SWD";
//
//    // IMU NWU -> IMU transform
//    Eigen::Quaternionf nwu_q_imu;
//
//    if (imu_frame_ori.compare("NWU")==0)
//        nwu_q_imu = Eigen::Quaternionf(1.0,0.0,0.0,0.0);
//    else if (imu_frame_ori.compare("ENU")==0)
//        nwu_q_imu = Eigen::Quaternionf(0.707,0.0,0.0,-0.707);
//    else if (imu_frame_ori.compare("SEU")==0)
//        nwu_q_imu = Eigen::Quaternionf(0.0,0.0,0.0,-1.0);
//    else if (imu_frame_ori.compare("WSU")==0)
//        nwu_q_imu = Eigen::Quaternionf(0.707,0.0,0.0,0.707);
//    else if (imu_frame_ori.compare("NED")==0)
//        nwu_q_imu = Eigen::Quaternionf(0.0,1.0,0.0,0.0);
//    else if (imu_frame_ori.compare("ESD")==0)
//        nwu_q_imu = Eigen::Quaternionf(0.0, 0.707,-0.707,0.0);
//    else if (imu_frame_ori.compare("SWD")==0)
//        nwu_q_imu = Eigen::Quaternionf(0.0, 0.0,-1.0,0.0);
//    else if (imu_frame_ori.compare("WND")==0)
//        nwu_q_imu = Eigen::Quaternionf(0.0, 0.707,0.707,0.0);
//    else
//        std::cerr << "[EskfOdomAlgNode]: IMU orientation not considered here. Possibilities: NW, ENU, SEU, WSU, NED, ESD, SWD, WND" << std::endl;
//
    Eigen::Translation<float,3> nwu_p_imu(0.0,0.0,0.0);
    TransformType nwuTimu = nwu_p_imu*nwu_q_imu;

    Eigen::Vector3f nwu_a = nwuTimu*a;
    Eigen::Vector3f nwu_w = nwuTimu*w;

    this->filter_.propagate_imu(dt,nwu_a,nwu_w);
}

// Sensor::pose_params EskfOdomAlgorithm::get_pose_params(void)
// {
//     // return this->filter_.get_pose_params();
// }

// Sensor::pose_params EskfOdomAlgorithm::get_pose2_params(void)
// {
//     // return this->filter_.get_pose2_params();
// }

// void EskfOdomAlgorithm::set_pose_params(Sensor::pose_params& pose)
// {
//     // this->filter_.set_pose_params(pose);
// }

// void EskfOdomAlgorithm::set_pose2_params(Sensor::pose_params& pose)
// {
//     // this->filter_.set_pose2_params(pose);
// }

void EskfOdomAlgorithm::set_pose_reading(const float& t, const Eigen::VectorXf& val)
{
    this->filter_.set_pose_reading(t,val);
}

void EskfOdomAlgorithm::set_pose2_reading(const float& t, const Eigen::VectorXf& val)
{
    // this->filter_.set_pose2_reading(t,val);
}

// Sensor::position_params EskfOdomAlgorithm::get_position_params(void)
// {
//     // return this->filter_.get_position_params();
// }

// void EskfOdomAlgorithm::set_position_params(Sensor::position_params& position)
// {
//     // this->filter_.set_position_params(position);
// }

void EskfOdomAlgorithm::set_position_reading(const float& t, const Eigen::Vector3f& val)
{
    // this->filter_.set_position_reading(t,val);
}

// Sensor::orientation_params EskfOdomAlgorithm::get_orientation_params(void)
// {
//     // return this->filter_.get_orientation_params();
// }

// void EskfOdomAlgorithm::set_orientation_params(Sensor::orientation_params& orientation)
// {
//     // this->filter_.set_orientation_params(orientation);
// }

void EskfOdomAlgorithm::set_orientation_reading(const float& t, const Eigen::Vector3f& val)
{
    // this->filter_.set_orientation_reading(t,val);
}

// Sensor::linvel_params EskfOdomAlgorithm::get_linvel_params(void)
// {
//     // return this->filter_.get_linvel_params();
// }

// void EskfOdomAlgorithm::set_linvel_params(Sensor::linvel_params& linvel)
// {
//     // this->filter_.set_linvel_params(linvel);
// }

void EskfOdomAlgorithm::set_linvel_reading(const float& t, const Eigen::Vector3f& val)
{
    // this->filter_.set_linvel_reading(t,val);
}

// Sensor::range_params EskfOdomAlgorithm::get_range_params(void)
// {
//     // return this->filter_.get_range_params();
// }

// void EskfOdomAlgorithm::set_range_params(Sensor::range_params& range)
// {
//     // this->filter_.set_range_params(range);
// }

void EskfOdomAlgorithm::set_range_reading(const float& t, const Eigen::VectorXf& val)
{
    // this->filter_.set_range_reading(t,val);
}

// Sensor::px4_params EskfOdomAlgorithm::get_px4_params(void)
// {
//     // return this->filter_.get_px4_params();
// }

// void EskfOdomAlgorithm::set_px4_params(Sensor::px4_params& px4)
// {
//     // this->filter_.set_px4_params(px4);
// }

void EskfOdomAlgorithm::set_px4_reading(const float& t, const Eigen::VectorXf& data)
{
    // this->filter_.set_px4_reading(t,data);
}

// Sensor::flow2d_params EskfOdomAlgorithm::get_flow2d_params(void)
// {
//     // return this->filter_.get_flow2d_params();
// }

// void EskfOdomAlgorithm::set_flow2d_params(Sensor::flow2d_params& flow2d)
// {
//     // this->filter_.set_flow2d_params(flow2d);
// }

// void EskfOdomAlgorithm::set_flow2d_reading(const float& t, const Eigen::Vector2f& val)
// {
//     // this->filter_.set_flow2d_reading(t,val);
// }

float EskfOdomAlgorithm::get_proc_time(void)
{
    // return this->filter_.get_proc_time();
}
bool EskfOdomAlgorithm::update(Eigen::VectorXf& state, Eigen::Vector3f& ang_vel, const bool& flying, const float& gnd_dist)
{
    // return this->filter_.update(state,ang_vel, flying, gnd_dist);
}
void EskfOdomAlgorithm::update(Eigen::VectorXf& state, Eigen::VectorXf& covPtrue)
{
    this->filter_.update(state, covPtrue);
}