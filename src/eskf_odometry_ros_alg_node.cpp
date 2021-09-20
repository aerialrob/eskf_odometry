#include "eskf_odometry_ros_alg_node.h"

// STD stuff
#include <fstream>
#include <iostream>
#include <sstream>
#include <random>

// TF stuff
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <eskf_odometry_ros/dxstate.h>
#include <eskf_odometry_ros/vP.h>
#include <eskf_odometry_ros/xstate.h>

EskfOdomAlgNode::EskfOdomAlgNode(void) :
algorithm_base::IriBaseAlgorithm<EskfOdomAlgorithm>()
{
    // Initialize all vars
    this->loop_rate_ = 1000; //in [Hz]
    this->seq_ = 0;
    this->flying_ = false;
    this->range_dist_ = Eigen::VectorXf::Zero(1); // a little bit more than the minimum distance PX4 can detect (0.3m)

    this->is_first_imu_ = true;
    this->is_first_range_ = true;        
    this->is_first_odom_ = true;
    this->is_first_odom2_ = true;
    this->is_first_position_ = true;

    // Initialize ROS Time objects
    this->ros_t_init_ = ros::Time::now();
    this->ros_t_last_ = this->ros_t_init_;
    this->ros_t_curr_ = this->ros_t_init_;

    // Read initial parameters from yaml file and set themn to sensors and filter
    read_and_set_ini_params();

    // [init publishers]
    this->odom_publisher_ = this->public_node_handle_.advertise<nav_msgs::Odometry>("odom_out", 1);
    this->state_publisher_ = this->public_node_handle_.advertise<eskf_odometry_ros::xstate>("xstate_out", 1);
    this->cov_publisher_ = this->public_node_handle_.advertise<eskf_odometry_ros::vP>("cov_out", 1);
    this->magnetic_publisher_ = this->public_node_handle_.advertise<sensor_msgs::MagneticField>("magnetic_msg", 1);

    // [init subscribers]
    this->position_in_subscriber_ = this->public_node_handle_.subscribe("position_in", 1, &EskfOdomAlgNode::position_in_callback, this);
    pthread_mutex_init(&this->position_in_mutex_,NULL);

    this->odom_subscriber_ = this->public_node_handle_.subscribe("odom_in", 1, &EskfOdomAlgNode::odom_callback, this);
    pthread_mutex_init(&this->odom_mutex_,NULL);

    this->odom2_subscriber_ = this->public_node_handle_.subscribe("odom2_in", 1, &EskfOdomAlgNode::odom2_callback, this);
    pthread_mutex_init(&this->odom2_mutex_,NULL);

    this->range_subscriber_ = this->public_node_handle_.subscribe("range", 1, &EskfOdomAlgNode::range_callback, this);
    pthread_mutex_init(&this->range_mutex_,NULL);

    this->imu_subscriber_ = this->public_node_handle_.subscribe("imu", 1, &EskfOdomAlgNode::imu_callback, this);
    pthread_mutex_init(&this->imu_mutex_, NULL);

    // [init services]
    this->flying_server_ = this->public_node_handle_.advertiseService("flying", &EskfOdomAlgNode::flyingCallback, this);
    pthread_mutex_init(&this->flying_mutex_,NULL);

    // [init clients]

    // [init action servers]

    // [init action clients]

    // Initialize Filter Thread
    Thread_ = boost::thread( boost::bind(&EskfOdomAlgNode::ThreadFunc, this) );
    RunThread_ = true;
}

EskfOdomAlgNode::~EskfOdomAlgNode(void)
{
    // [free dynamic memory]
    pthread_mutex_destroy(&this->position_in_mutex_);
    pthread_mutex_destroy(&this->odom_mutex_);
    pthread_mutex_destroy(&this->odom2_mutex_);
    pthread_mutex_destroy(&this->range_mutex_);
    pthread_mutex_destroy(&this->flying_mutex_);    
    pthread_mutex_destroy(&this->imu_mutex_);

    RunThread_ = false;
    Thread_.join();
}

void EskfOdomAlgNode::read_and_set_ini_params(void)
{
    // Frame names
    this->public_node_handle_.param<std::string>("robot_frame_id", this->robot_frame_id_, "");
    this->public_node_handle_.param<std::string>("world_frame_id", this->world_frame_id_, "");
    this->public_node_handle_.param<std::string>("odom_out_frame_id", this->odom_out_frame_id_, "");
    this->public_node_handle_.param<std::string>("odom_in_frame_id", this->odom_in_frame_id_, "");
    this->public_node_handle_.param<std::string>("odom2_in_frame_id", this->odom2_in_frame_id_, "");
    this->public_node_handle_.param<std::string>("imu_frame_id",imu_frame_id_,"");

    // Get relative orientations between robot_base_link (assumed NWU) and the sensors
    TransformType r_T_imu = get_tf_transform(robot_frame_id_, imu_frame_id_);
    nwu_q_imu_ = Eigen::Quaternionf(r_T_imu.rotation());

    // TransformType r_T_flow = get_tf_transform(robot_frame_id_, flow_frame_id_);
    // nwu_q_flow_ = Eigen::Quaternionf(r_T_flow.rotation());

    // TransformType w_T_odom = get_tf_transform(world_frame_id_, odom_in_frame_id_);
    // nwu_q_odomin_ = Eigen::Quaternionf(w_T_odom.rotation());

    // TransformType w_T_odom2 = get_tf_transform(world_frame_id_, odom2_in_frame_id_);
    // nwu_q_odom2in_ = Eigen::Quaternionf(w_T_odom2.rotation());

    // Note that the odometry rotation offset should be computed the first time we receive an odometry,
    // because it usually requires to have a close TF tree,
    // which is done with our first publication of odom estimation.

    // Initial STD Error-State values (filter parameters)
    params f_params;
    f_params.dp0_std = read_vec("dp0_std", 3);
    f_params.dv0_std = read_vec("dv0_std", 3);
    f_params.dtheta0_std = read_vec("dtheta0_std", 3);
    f_params.dab0_std = read_vec("dab0_std", 3);
    f_params.dwb0_std = read_vec("dwb0_std", 3);
    f_params.dg0_std = read_vec("dg0_std", 3);
    f_params.dpo0_std = read_vec("dpo0_std", 3);
    f_params.dthetao0_std = read_vec("dthetao0_std", 3);
    f_params.dpr0_std = read_vec("dpr0_std", 3);
    f_params.dthetar0_std = read_vec("dthetar0_std", 3);

    std::string frame; // Frame r.t. orientation derivation is computed (g:global, l:local)
    this->public_node_handle_.param<std::string>("frame", frame, "global");
    if (frame == "global")
        f_params.frame = 'g';
    else if (frame == "local")
        f_params.frame = 'l';
    else
        std::cout << "ERROR loading frame: Wrong frame name (should be local or global)" << std::endl;

    // IMU STD values
    Sensor::imu_params imu_params;
    imu_params.std.segment(0,3) = read_vec("imu_a_std", 3);
    imu_params.std.segment(3,3) = read_vec("imu_w_std", 3);
    imu_params.std.segment(6,3) = read_vec("imu_ab_std", 3);
    imu_params.std.segment(9,3) = read_vec("imu_wb_std", 3);

    std::string imu_met;
    this->public_node_handle_.param<std::string>("imu_method", imu_met, "continuous");
    if (imu_met == "continuous")
        imu_params.met = 'c';
    else if (imu_met == "discrete")
        imu_params.met = 'd';
    else
        std::cout << "ERROR loading frame: Wrong IMU method name (should be continuous or discrete)" << std::endl;

    // Position STD values
    Sensor::position_params position_params;
    position_params.std_outsidebounds = read_vec("position_std_outsidebounds", 3);
    position_params.std_insidebounds = read_vec("position_std_insidebounds", 3);
    position_params.std = position_params.std_insidebounds;

    // Orientation STD values
    Sensor::orientation_params orientation_params;
    orientation_params.std_outsidebounds = read_vec("orientation_std_outsidebounds", 3);
    orientation_params.std_insidebounds = read_vec("orientation_std_insidebounds", 3);
    orientation_params.std = orientation_params.std_insidebounds;

    // Linear Velocity STD values
    Sensor::linvel_params linvel_params;
    linvel_params.std_outsidebounds = read_vec("linvel_std_outsidebounds", 3);
    linvel_params.std_insidebounds = read_vec("linvel_std_insidebounds", 3);
    linvel_params.std = linvel_params.std_insidebounds;

    //Odometry (pose + twist) STD values
    Sensor::pose_params pose_params;
    pose_params.std_outsidebounds = read_vec("pose_std_outsidebounds", 6);
    pose_params.std_insidebounds = read_vec("pose_std_insidebounds", 6);
    pose_params.std = pose_params.std_insidebounds;

    //Odometry 2 (pose + twist) STD values
    Sensor::pose_params pose2_params;
    pose2_params.std_outsidebounds = read_vec("pose2_std_outsidebounds", 6);
    pose2_params.std_insidebounds = read_vec("pose2_std_insidebounds", 6);
    pose2_params.std = pose2_params.std_insidebounds;

    // Range STD values
    // Sensor::range_params range_params;
    // double std_outsidebounds, std_insidebounds;
    // this->public_node_handle_.param<double>("range_std_outsidebounds", std_outsidebounds, 10.0);
    // this->public_node_handle_.param<double>("range_std_insidebounds", std_insidebounds, 0.01);
    // range_params.std_outsidebounds(0) = static_cast<float>(std_outsidebounds);
    // range_params.std_insidebounds(0) = static_cast<float>(std_insidebounds);
    // range_params.std = range_params.std_outsidebounds; // Considering initially landed
    // double range_min, range_max;
    // this->public_node_handle_.param<double>("range_min", range_min, 4.0);
    // this->public_node_handle_.param<double>("range_max", range_max, 0.31);
    // range_params.range_min = static_cast<float>(range_min);
    // range_params.range_max = static_cast<float>(range_max);
    // this->public_node_handle_.param<bool>("using_laser_range", range_params.using_laser, true);

    // Initialize Nominal-State vector
    Eigen::VectorXf x_state = read_vec("xstate0", 19);

    // Initialize Error-State vector
    Eigen::VectorXf dx_state = read_vec("dxstate0", 18);

    this->alg_.lock();  // protect algorithm

    // Set filter and sensors initial parameters
    // this->alg_.set_init_params(f_params, x_state, dx_state, imu_params, pose_params, pose2_params, position_params, orientation_params, linvel_params, range_params, px4_params, flow2d_params);
    // this->alg_.set_init_params(f_params, x_state, dx_state, imu_params, pose_params, pose2_params, position_params, orientation_params, linvel_params);
    this->alg_.set_init_params(f_params, x_state, dx_state, imu_params, position_params, orientation_params);
    

    // Print already set values of filter and sensors initial parameters
    this->alg_.print_ini_params();

    this->alg_.unlock();
}

void show_result_info (const int result, const std::string sensor)
{
    if (result == -1)
        ROS_ERROR_STREAM( "MSG " << sensor << " Not processed, first imu msg not received  ");
    else if (result == 0)
        ROS_WARN_STREAM( "MSG " << sensor << " too old to process ");
    else if (result == 1)
        ROS_INFO_STREAM( "MSG " << sensor << " success ");
}


Eigen::VectorXf EskfOdomAlgNode::read_vec(const std::string& param_name,const int& exp_long)
{
    Eigen::VectorXd params = Eigen::VectorXd::Zero(exp_long);

    XmlRpc::XmlRpcValue my_list;
    this->public_node_handle_.getParam(param_name, my_list);

    if (my_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        int vec_size = int(my_list.size());
        if (vec_size == exp_long)
        {
            for (int ii = 0; ii < vec_size; ii++)
                params(ii) = my_list[ii];
        }
        else
            std::cout << "ERROR loading " << param_name << ": Wrong elements number" << std::endl;
    }
    else
        std::cout << "ERROR loading " << param_name << ": Wrong element type" << std::endl;

    Eigen::VectorXf paramsf = params.cast<float>();
    return paramsf;
}

void EskfOdomAlgNode::mainNodeThread(void)
{
    // [fill msg structures]

    // [fill srv structure and make request to the server]

    // [fill action structure and make request to the action server]

    // [publish messages]
}

void EskfOdomAlgNode::ThreadFunc(void)
{
    ros::Rate rate(20);
    while(RunThread_)
    {
        // DEBUG: Frequency check
        this->ros_t_curr_ = ros::Time::now();
        double t_last = (this->ros_t_last_ - this->ros_t_init_).toSec();
        double t_curr = (this->ros_t_curr_ - this->ros_t_init_).toSec();
        double dt = t_curr - t_last;
        this->ros_t_last_ = this->ros_t_curr_;
        // std::cout << "Current freq: " << 1/dt << " Hz" << std::endl;

        this->flying_mutex_enter();
        bool is_flying = this->flying_;
        this->flying_mutex_exit();

        Eigen::VectorXf state(33,1);
        Eigen::VectorXf vcovP(19,1);  
        eskf_odometry_ros::vP ros_vcovP;
        eskf_odometry_ros::xstate ros_state;
        eskf_odometry_ros::xstate dxstate;

        Eigen::Vector3f ang_vel;
        bool step_done = false;

        this->alg_.lock();
        // step_done = this->alg_.update(state,ang_vel,is_flying,this->range_dist_(0));
        this->alg_.update(state, vcovP);
        ros_state.p.x = state(0);
        ros_state.p.y = state(1);
        ros_state.p.z = state(2);
        ros_state.v.x = state(3);
        ros_state.v.y = state(4);
        ros_state.v.z = state(5);
        ros_state.q.w = state(6);
        ros_state.q.x = state(7);
        ros_state.q.y = state(8);
        ros_state.q.z = state(9);
        ros_state.ab.x = state(10);
        ros_state.ab.y = state(11);
        ros_state.ab.z = state(12);
        ros_state.wb.x = state(13);
        ros_state.wb.y = state(14);
        ros_state.wb.z = state(15);
        ros_state.g.x = state(16);
        ros_state.g.y = state(17);
        ros_state.g.z = state(18);

        ros_vcovP.p.x = vcovP(0);
        ros_vcovP.p.y = vcovP(1);
        ros_vcovP.p.z = vcovP(2);
        ros_vcovP.v.x = vcovP(3);
        ros_vcovP.v.y = vcovP(4);
        ros_vcovP.v.z = vcovP(5);
        ros_vcovP.q.w = vcovP(6);
        ros_vcovP.q.x = vcovP(7);
        ros_vcovP.q.y = vcovP(8);
        ros_vcovP.q.z = vcovP(9);
        ros_vcovP.ab.x = vcovP(10);
        ros_vcovP.ab.y = vcovP(11);
        ros_vcovP.ab.z = vcovP(12);
        ros_vcovP.wb.x = vcovP(13);
        ros_vcovP.wb.y = vcovP(14);
        ros_vcovP.wb.z = vcovP(15);
        ros_vcovP.g.x = vcovP(16);
        ros_vcovP.g.y = vcovP(17);
        ros_vcovP.g.z = vcovP(18);
        

        this->alg_.unlock();
        ros::spinOnce();
        rate.sleep();

        this->state_publisher_.publish(ros_state);
        this->cov_publisher_.publish(ros_vcovP);



        // std::cout << "RunThread_ "  << std::endl;

        // if (step_done)
        // {
        //     // Broadcast state with TF
        //     static tf::TransformBroadcaster br;
        //     tf::Transform transform;
        //     transform.setOrigin(tf::Vector3(state(0), state(1), state(2)));
        //     tf::Quaternion q(state(7), state(8), state(9), state(6)); //TF:[qx,qy,qz,qw] Filter:[qw,qx,qy,qz]
        //     transform.setRotation(q);
        //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->odom_out_frame_id_, this->robot_frame_id_));

        //     // Publish Odometry
        //     this->odom_msg_.header.seq = this->seq_;
        //     this->odom_msg_.header.stamp = ros::Time::now();
        //     this->odom_msg_.header.frame_id = this->odom_out_frame_id_;
        //     this->odom_msg_.child_frame_id = this->robot_frame_id_;
        //     this->odom_msg_.pose.pose.position.x = state(0);
        //     this->odom_msg_.pose.pose.position.y = state(1);
        //     this->odom_msg_.pose.pose.position.z = state(2);
        //     this->odom_msg_.pose.pose.orientation.w = state(6);
        //     this->odom_msg_.pose.pose.orientation.x = state(7);
        //     this->odom_msg_.pose.pose.orientation.y = state(8);
        //     this->odom_msg_.pose.pose.orientation.z = state(9);
        //     this->odom_msg_.twist.twist.linear.x = state(3);
        //     this->odom_msg_.twist.twist.linear.y = state(4);
        //     this->odom_msg_.twist.twist.linear.z = state(5);
        //     this->odom_msg_.twist.twist.angular.x = ang_vel(0);
        //     this->odom_msg_.twist.twist.angular.y = ang_vel(1);
        //     this->odom_msg_.twist.twist.angular.z = ang_vel(2);
        //     this->odom_publisher_.publish(this->odom_msg_);
        //     ++this->seq_;
        // }
    }
}

/*  [subscriber callbacks] */
void EskfOdomAlgNode::position_in_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // ROS_DEBUG("EskfOdomAlgNode::position_in_callback: New Message Received");

    // this->position_in_mutex_enter();

    // ros::Time stamp = msg->header.stamp;
    // double t = stamp.toSec();
    // if (this->is_first_position_)
    // {
    //     this->t_ini_position_ = t;
    //     this->is_first_position_ = false;
    // }

    // this->position_in_mutex_exit();

    // // get minimum height when landed to avoid -Z outliers (e.g. due to GPS wrong heights)
    // this->alg_.lock();
    // double range_min = this->alg_.get_range_params().range_min;
    // this->alg_.unlock();
    // if (!std::isfinite(range_min)) // Only set if known
    //     range_min = 0.0;
    
    // Eigen::Vector3f position;
    // position(0) = msg->point.x;
    // position(1) = msg->point.y;
    // if (this->flying_ && msg->point.z > range_min)
    //     position(2) = msg->point.z;
    // else
    //     position(2) = this->odom_msg_.pose.pose.position.z; // In case of not flying, update only x,y (avoiding -z and jumps)

    // this->alg_.lock();
    // this->alg_.set_position_reading(static_cast<float>(t-this->t_ini_position_), position); // Set values into filter object
    // this->alg_.unlock();
}

void EskfOdomAlgNode::position_in_mutex_enter(void)
{
    pthread_mutex_lock(&this->position_in_mutex_);
}

void EskfOdomAlgNode::position_in_mutex_exit(void)
{
    pthread_mutex_unlock(&this->position_in_mutex_);
}

void EskfOdomAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("EskfOdomAlgNode::odom_callback: New Message Received");

    this->odom_mutex_enter();

    ros::Time stamp = msg->header.stamp;
    // double t = stamp.toSec();
    ros::Time begin  = ros::Time::now();  
    double t = begin.toSec(); 
    if (this->is_first_odom_)
    {
        this->t_ini_odom_ = t;        
        this->is_first_odom_ = false;
    }

    this->odom_mutex_exit();

    // get minimum height when landed to avoid -Z outliers (e.g. due to GPS wrong heights)
    // this->alg_.lock();
    // double range_min = this->alg_.get_range_params().range_min;
    // this->alg_.unlock();
    // if (!std::isfinite(range_min)) // Only set if known
    //     range_min = 0.0;

    Eigen::Vector3f pose;    // Position x,y,z
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;
    // if (this->flying_ && msg->pose.pose.position.z > range_min)
    //     pose(2) = msg->pose.pose.position.z;
    // else
    //     pose(2) = this->odom_msg_.pose.pose.position.z; // In case of not flying, update only x,y (avoiding -z and jumps)

    // double roll, pitch, yaw;
    // tf::Quaternion qt;
    // tf::quaternionMsgToTF(msg->pose.pose.orientation,qt);
    // tf::Matrix3x3(qt).getRPY(roll,pitch,yaw);
    // pose.segment(3,3) << roll,pitch,yaw;

    // Transform to NWU frame (required by low level library)
    // pose.segment(0,3) = nwu_q_odomin_ * pose.segment(0,3);
    // pose.segment(3,3) = nwu_q_odomin_ * pose.segment(3,3);

    Eigen::Matrix3f R_position; 
    
    R_position << msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2],
         msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[8],
         msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14];
    ROS_INFO_STREAM("R_position " << R_position);
    this->alg_.lock();
    int result;
    result = this->alg_.set_position_reading(static_cast<float>(t-this->t_ini_imu_), pose, R_position); // Use diff dt    
    show_result_info(result, "---POSE");
    this->alg_.unlock();    



    Eigen::Vector3f magnetic_field_msg;    
    Eigen::Vector3f magnetic_field_vector{1,1,1};    
    
    // Get orientation msg
    Eigen::Quaternionf qt{msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z};
    Eigen::Matrix3f R_temp;
    R_temp = qt.matrix();

    // Define random generator with Gaussian distribution
    const double mean = 0.0;
    const double stddev = 0.5;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    Eigen::Vector3f noise{dist(generator), dist(generator), dist(generator)};     

    // Magnetic field msg 
    magnetic_field_msg = R_temp.transpose() * magnetic_field_vector + noise; 

    ROS_INFO_STREAM("magnetic_field_msg " << magnetic_field_msg);
       
    Eigen::Matrix3f Rmag;     
    Rmag << msg->pose.covariance[21], msg->pose.covariance[22], msg->pose.covariance[23],
            msg->pose.covariance[27], msg->pose.covariance[28], msg->pose.covariance[29],
            msg->pose.covariance[33], msg->pose.covariance[34], msg->pose.covariance[35];

    sensor_msgs::MagneticField mag_msg;
    mag_msg.magnetic_field.x = magnetic_field_msg[0];
    mag_msg.magnetic_field.y = magnetic_field_msg[1];
    mag_msg.magnetic_field.z = magnetic_field_msg[2];
    mag_msg.magnetic_field_covariance[0] = 0.05;
    mag_msg.magnetic_field_covariance[4] = 0.02;
    mag_msg.magnetic_field_covariance[8] = 0.03;
    magnetic_publisher_.publish(mag_msg);            

    this->alg_.lock();
    result = this->alg_.set_magnetometer_reading(static_cast<float>(t-this->t_ini_imu_), magnetic_field_msg, Rmag); // Use diff dt    
    show_result_info(result, "---MAGNETIC");
    this->alg_.unlock();  

}

void EskfOdomAlgNode::odom_mutex_enter(void)
{
    pthread_mutex_lock(&this->odom_mutex_);
}

void EskfOdomAlgNode::odom_mutex_exit(void)
{
    pthread_mutex_unlock(&this->odom_mutex_);
}

void EskfOdomAlgNode::odom2_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_DEBUG("EskfOdomAlgNode::odom2_callback: New Message Received");

    // this->odom2_mutex_enter();

    // ros::Time stamp = msg->header.stamp;
    // double t = stamp.toSec();
    // if (this->is_first_odom2_)
    // {
    //     this->t_ini_odom2_ = t;
    //     this->is_first_odom2_ = false;
    // }

    // this->odom2_mutex_exit();

    // // get minimum height when landed to avoid -Z outliers (e.g. due to GPS wrong heights)
    // this->alg_.lock();
    // double range_min = this->alg_.get_range_params().range_min;
    // this->alg_.unlock();
    // if (!std::isfinite(range_min)) // Only set if known
    //     range_min = 0.0;

    // Eigen::VectorXf pose(6);
    // pose(0) = msg->pose.pose.position.x;
    // pose(1) = msg->pose.pose.position.y;
    // if (this->flying_ && msg->pose.pose.position.z > range_min)
    //     pose(2) = msg->pose.pose.position.z;
    // else
    //     pose(2) = this->odom_msg_.pose.pose.position.z; // In case of not flying, update only x,y (avoiding -z and jumps)

    // double roll, pitch, yaw;
    // tf::Quaternion qt;
    // tf::quaternionMsgToTF(msg->pose.pose.orientation,qt);
    // tf::Matrix3x3(qt).getRPY(roll,pitch,yaw);
    // pose.segment(3,3) << roll,pitch,yaw;

    // // Transform to NWU frame (required by low level library)
    // pose.segment(0,3) = nwu_q_odomin_ * pose.segment(0,3);
    // pose.segment(3,3) = nwu_q_odomin_ * pose.segment(3,3);

    // this->alg_.lock();
    // this->alg_.set_pose2_reading(static_cast<float>(t-this->t_ini_odom_), pose); // Set values into filter object
    // this->alg_.unlock();
}

void EskfOdomAlgNode::odom2_mutex_enter(void)
{
    pthread_mutex_lock(&this->odom2_mutex_);
}

void EskfOdomAlgNode::odom2_mutex_exit(void)
{
    pthread_mutex_unlock(&this->odom2_mutex_);
}

void EskfOdomAlgNode::range_callback(const sensor_msgs::Range::ConstPtr& msg)
{
    // ROS_DEBUG("EskfOdomAlgNode::range_callback: New Message Received");

    // this->range_mutex_enter();
    // ros::Time stamp = msg->header.stamp;
    // double t = stamp.toSec();
    // if (this->is_first_range_)
    // {
    //     this->t_ini_range_ = t;
    //     this->is_first_range_ = false;
    // }
    // this->range_dist_(0) = msg->range;

    // // // OPTIONAL: Set the range limits dynamically
    // // Sensor::range_params range_params = this->alg_.get_range_params();
    // // if (std::isfinite(msg->min_range)) // Only set if known
    // //   range_params.range_min = msg->min_range;
    // // if (std::isfinite(msg->max_range)) // Only set if known
    // //   range_params.range_min = msg->max_range;
    // // this->alg_.lock();
    // // this->alg_.set_range_params(range_params);
    // // this->alg_.unlock();

    // this->range_mutex_exit();

    // this->alg_.lock();
    // this->alg_.set_range_reading(static_cast<float>(t-this->t_ini_range_), this->range_dist_); // Set values into filter object
    // this->alg_.unlock();
}

void EskfOdomAlgNode::range_mutex_enter(void)
{
    pthread_mutex_lock(&this->range_mutex_);
}

void EskfOdomAlgNode::range_mutex_exit(void)
{
    pthread_mutex_unlock(&this->range_mutex_);
}

void EskfOdomAlgNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    /// Guardar el tiempo de la última medida y de su anterior
    /// w ultima a ultima t medida anterior 

    ROS_DEBUG("EskfOdomAlgNode::imu_callback: New Message Received");

    this->imu_mutex_enter();
    ros::Time stamp = msg->header.stamp;

    // TODO select rostime from msg or from node time 
    // double t = stamp.toSec();
    // time now
    ros::Time begin  = ros::Time::now();  
    double t = begin.toSec(); 

    if (this->is_first_imu_)
    {
        this->t_prev_imu_ = t;  
        this->t_ini_imu_ = t;
        this->is_first_imu_ = false;
    }
    this->imu_mutex_exit();

    set_imu_reading(msg, static_cast<float>(t-this->t_ini_imu_));        // t from 0
    // set_imu_reading(msg, static_cast<float>(t - this->t_prev_imu_));        // Use diff dt
    // set_imu_reading(msg, static_cast<float>(t));        // Use diff dt

    // // Update Prev INI sensor time.
    // this->t_prev_imu_ = t;
}
void EskfOdomAlgNode::imu_mutex_enter(void)
{
    pthread_mutex_lock(&this->imu_mutex_);
}
void EskfOdomAlgNode::imu_mutex_exit(void)
{
    pthread_mutex_unlock(&this->imu_mutex_);
}

void EskfOdomAlgNode::set_imu_reading(const sensor_msgs::Imu::ConstPtr& msg, const float& t_msg)
{
    this->imu_mutex_enter();
    float ax, ay, az, wx, wy, wz;
    ax = msg->linear_acceleration.x;
    ay = msg->linear_acceleration.y;
    az = msg->linear_acceleration.z;
    wx = msg->angular_velocity.x;
    wy = msg->angular_velocity.y;
    wz = msg->angular_velocity.z;
    this->imu_mutex_exit();

    // Guarda como globales para otros callbacks
    // Funcion para guardar tiempo actual, tiempo anterior a y w


    // Propagate
    // Crear propagate or Update

    Eigen::Vector3f a;
    a << ax, ay, az;
    Eigen::Vector3f w;
    w << wx, wy, wz;
    Eigen::Quaternionf q(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);

    this->alg_.lock();

    // // OPTIONAL: USE Actual IMU covariance. If first element is zero, then no covariance set.
    // if (msg->angular_velocity_covariance[0]>0.0 && msg->linear_acceleration_covariance[0]>0.0)
    // {
    //   Sensor::imu_params imu = this->alg_.get_imu_params();
    //   imu_w_std << msg->angular_velocity_covariance[0],msg->angular_velocity_covariance[4],msg->angular_velocity_covariance[8];
    //   imu_a_std << msg->linear_acceleration_covariance[0],msg->linear_acceleration_covariance[4],msg->linear_acceleration_covariance[8];
    //   this->alg_.set_imu_params(imu);
    // }
    // ROS_INFO_STREAM("t_msg " << t_msg);


    Eigen::Matrix3f Rw; 
    
    // Rw << msg->angular_velocity_covariance[0], msg->angular_velocity_covariance[1], msg->angular_velocity_covariance[2],
    //       msg->angular_velocity_covariance[3], msg->angular_velocity_covariance[4], msg->angular_velocity_covariance[5],
    //       msg->angular_velocity_covariance[6], msg->angular_velocity_covariance[7], msg->angular_velocity_covariance[8];

    Rw << 0.0005, msg->angular_velocity_covariance[1], msg->angular_velocity_covariance[2],
          msg->angular_velocity_covariance[3], 0.0005, msg->angular_velocity_covariance[5],
          msg->angular_velocity_covariance[6], msg->angular_velocity_covariance[7], 0.0005;          

    Eigen::Matrix3f Ra; 
    
    // Ra << msg->linear_acceleration_covariance[0], msg->linear_acceleration_covariance[1], msg->linear_acceleration_covariance[2],
    //       msg->linear_acceleration_covariance[3], msg->linear_acceleration_covariance[4], msg->linear_acceleration_covariance[5],
    //       msg->linear_acceleration_covariance[6], msg->linear_acceleration_covariance[7], msg->linear_acceleration_covariance[8];  

    Ra << 0.03, msg->linear_acceleration_covariance[1], msg->linear_acceleration_covariance[2],
        msg->linear_acceleration_covariance[3], 0.03, msg->linear_acceleration_covariance[5],
        msg->linear_acceleration_covariance[6], msg->linear_acceleration_covariance[7], 0.03;  

    ROS_INFO_STREAM("Ra " << Ra);                 
    ROS_INFO_STREAM("Rw " << Rw);            

    int result;
    result = this->alg_.set_imu_reading(t_msg, a, w, Rw, Ra, nwu_q_imu_); // To low level propagate
    // Show console result
    show_result_info(result, "IMU");

    this->alg_.unlock();
}

/*  [service callbacks] */
bool EskfOdomAlgNode::flyingCallback(roscpp::Empty::Request &req, roscpp::Empty::Response &res)
{
    // ROS_DEBUG("EskfOdomAlgNode::flyingCallback: New Request Received!");

    // this->flying_mutex_enter();
    // if (this->flying_)
    //     this->flying_ = false;
    // else
    //     this->flying_ = true;
    // this->flying_mutex_exit();
    // return true;
}

void EskfOdomAlgNode::flying_mutex_enter(void)
{
    pthread_mutex_lock(&this->flying_mutex_);
}

void EskfOdomAlgNode::flying_mutex_exit(void)
{
    pthread_mutex_unlock(&this->flying_mutex_);
}

TransformType EskfOdomAlgNode::get_tf_transform(const std::string& parent_frame, const std::string& child_frame)
{
    bool transform_ok = false;

    tf::TransformListener listener;
    tf::StampedTransform tf_trans;

    if (child_frame.compare("")!=0) // Only look for transform if child_frame_id is provided. Otherwise we consider the data is in the correct frame.
    {
        ros::Time tnow = ros::Time::now();

        while (!transform_ok)
        {
            try
            {
                listener.waitForTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform (parent_frame, child_frame, ros::Time(0), tf_trans);
                transform_ok = true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("[EskfOdomAlgNode]: Cannot get tf transform: %s", ex.what());
            }
        }
    }
    else
    {
        tf::Transform Identity;
        Identity.setIdentity();
        tf_trans.setData(Identity);
    }

    // tf transform to Eigen homogenous matrix
    return tf_to_eigen(tf_trans);
}

TransformType EskfOdomAlgNode::tf_to_eigen(const tf::Transform& _transform)
{
    double roll, pitch, yaw;
    _transform.getBasis().getRPY(roll, pitch, yaw);

    // Euler ZYX convention
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf((float)yaw,   Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf((float)pitch, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf((float)roll,  Eigen::Vector3f::UnitX());

    Eigen::Translation<float,3> p(_transform.getOrigin().x(),_transform.getOrigin().y(),_transform.getOrigin().z());

    TransformType eigenT = p*q;

    return eigenT;
}

/*  [action callbacks] */

/*  [action requests] */

void EskfOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
    this->alg_.lock();

    this->alg_.unlock();
}

void EskfOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
    return algorithm_base::main<EskfOdomAlgNode>(argc, argv, "eskf_odom_alg_node");
}
