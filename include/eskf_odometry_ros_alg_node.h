// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _eskf_odom_alg_node_h_
#define _eskf_odom_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "eskf_odometry_ros_alg.h"

// STD stuff
#include <string>

// Eigen stuff
#include <eigen3/Eigen/Dense>

// Boost stuff
#include <boost/thread/thread.hpp>

// ROS
#include <tf/transform_listener.h>

// [publisher subscriber headers]
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <px_comm/OpticalFlow.h>
#include <sensor_msgs/Imu.h>

// [service client headers]
#include <roscpp/Empty.h>

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class EskfOdomAlgNode: public algorithm_base::IriBaseAlgorithm<EskfOdomAlgorithm> {

    private:



        ros::Time ros_t_init_; // Initial ROS time.
        ros::Time ros_t_last_; // ROS time in last loop step.
        ros::Time ros_t_curr_; // ROS time in current loop step.

        bool flying_; // Quadrotor landed or flying information.
        Eigen::VectorXf range_dist_; // Ground distance (m) obtained from PX4 optical flow pointing downward.

        bool is_first_imu_;        // First IMU reading should not contribute to propagate nominal (integration requirements). Also used to get initial sensor time.
        bool is_first_range_;      // Used to get initial sensor time.              
        bool is_first_odom_;       // Used to get initial sensor time.
        bool is_first_odom2_;       // Used to get initial sensor time.
        bool is_first_position_;   // Used to get initial sensor time.

        double t_ini_imu_;    // Initial IMU sensor time.
        double t_ini_range_;  // Initial RANGE sensor time.              
        double t_ini_odom_;   // Initial Odome sensor time.
        double t_ini_odom2_;   // Initial Odome sensor time.
        double t_ini_position_;   // Initial Odome sensor time.

        std::string imu_frame_ori_; //Indicates the IMU frame orientation.

        int seq_; // Odometry sequence counter.
        std::string world_frame_id_; // World frame ID.
        std::string robot_frame_id_; // Robot frame ID.
        std::string odom_in_frame_id_; // Odometry IN frame ID.
        std::string odom2_in_frame_id_; // Odometry 2 IN frame ID.
        std::string odom_out_frame_id_; // Odometry OUT frame ID.
        std::string imu_frame_id_; // IMU frame ID.
        std::string flow_frame_id_; // Optical flow sensor frame ID.
        Eigen::Quaternionf nwu_q_imu_; // Rotation of frame NWU w.r.t. current IMU frame (i.e. expressed in current IMU frame).
        Eigen::Quaternionf nwu_q_flow_; // Rotation of frame NWU w.r.t. current FLOW frame (i.e. expressed in current FLOW frame).
        Eigen::Quaternionf nwu_q_odomin_; // Rotation of frame NWU w.r.t. current ODOM frame (i.e. expressed in current FLOW frame).
        Eigen::Quaternionf nwu_q_odom2in_; // Rotation of frame NWU w.r.t. current ODOM 2 frame (i.e. expressed in current FLOW frame).

        bool got_odom_tf_offset_; // Flag to get only the first tf transform when starts running and the tf tree is complete.

        bool px4_use_vxy_; // Use Vxy update. Flase uses Flow2D.

        // Processing thread
        boost::thread Thread_;
        bool RunThread_;
        void ThreadFunc(void);

        // [publisher attributes]
        ros::Publisher odom_publisher_;
        ros::Publisher state_publisher_;
        ros::Publisher cov_publisher_;
        nav_msgs::Odometry odom_msg_;

        // [subscriber attributes]
        ros::Subscriber position_in_subscriber_;
        void position_in_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
        pthread_mutex_t position_in_mutex_;
        void position_in_mutex_enter(void);
        void position_in_mutex_exit(void);

        ros::Subscriber odom_subscriber_;
        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        pthread_mutex_t odom_mutex_;
        void odom_mutex_enter(void);
        void odom_mutex_exit(void);

        ros::Subscriber odom2_subscriber_;
        void odom2_callback(const nav_msgs::Odometry::ConstPtr& msg);
        pthread_mutex_t odom2_mutex_;
        void odom2_mutex_enter(void);
        void odom2_mutex_exit(void);

        ros::Subscriber range_subscriber_;
        void range_callback(const sensor_msgs::Range::ConstPtr& msg);
        pthread_mutex_t range_mutex_;
        void range_mutex_enter(void);
        void range_mutex_exit(void);

        ros::Subscriber imu_subscriber_;
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        pthread_mutex_t imu_mutex_;
        void imu_mutex_enter(void);
        void imu_mutex_exit(void);
        void set_imu_reading(const sensor_msgs::Imu::ConstPtr& msg,const float& msg_time);

        // [service attributes]
        ros::ServiceServer flying_server_;
        bool flyingCallback(roscpp::Empty::Request &req, roscpp::Empty::Response &res);
        pthread_mutex_t flying_mutex_;
        void flying_mutex_enter(void);
        void flying_mutex_exit(void);

        // [client attributes]

        // [action server attributes]

        // [action client attributes]

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * \brief Constructor
         *
         * This constructor initializes specific class attributes and all ROS
         * communications variables to enable message exchange.
         */
        EskfOdomAlgNode(void);

        /**
         * \brief Destructor
         *
         * This destructor frees all necessary dynamic memory allocated within this
         * this class.
         */
        ~EskfOdomAlgNode(void);

    protected:
        /**
         * \brief main node thread
         *
         * This is the main thread node function. Code written here will be executed
         * in every node loop while the algorithm is on running state. Loop frequency
         * can be tuned by modifying loop_rate attribute.
         *
         * Here data related to the process loop or to ROS topics (mainly data structs
         * related to the MSG and SRV files) must be updated. ROS publisher objects
         * must publish their data in this process. ROS client servers may also
         * request data to the corresponding server topics.
         */
        void mainNodeThread(void);

        /**
         * \brief dynamic reconfigure server callback
         *
         * This method is called whenever a new configuration is received through
         * the dynamic reconfigure. The derivated generic algorithm class must
         * implement it.
         *
         * \param config an object with new configuration from all algorithm
         *               parameters defined in the config file.
         * \param level  integer referring the level in which the configuration
         *               has been changed.
         */
        void node_config_update(Config &config, uint32_t level);

        /**
         * \brief node add diagnostics
         *
         * In this abstract function additional ROS diagnostics applied to the
         * specific algorithms may be added.
         */
        void addNodeDiagnostics(void);

        // [diagnostic functions]

        // [test functions]

        /**
         * \brief Read Parameters
         *
         * This function reads the parameters from the YAML file specified in the launch file
         */
        void read_and_set_ini_params(void);

        /**
         * \brief Read 3-Vector parameter
         *
         * This function reads a 3-Vector parameter specified by param_name
         */
        Eigen::VectorXf read_vec(const std::string& param_name, const int& exp_long);

        /**
         * \brief Get TF transform. Use a tf listener to obtain relative transform: child expressed in parent's coordinate frame.
         */
        TransformType get_tf_transform(const std::string& parent_frame, const std::string& child_frame);

        /**
         * \brief Get Eigen homogenous transform
         */
        TransformType tf_to_eigen(const tf::Transform& _transform);
};

#endif
