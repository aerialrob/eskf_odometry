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

#ifndef _eskf_odom_alg_h_
#define _eskf_odom_alg_h_

#include <eskf_odometry_ros/EskfOdomConfig.h>

// Main eskf library
#include <eskf_odometry.h>

// Eigen
#include <Eigen/Dense>

typedef Eigen::Transform<float, 3, Eigen::Affine> TransformType;

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class EskfOdomAlgorithm
{
    protected:
        /**
         * \brief define config type
         *
         * Define a Config type with the EskfOdomConfig. All driver implementations
         * will then use the same variable type Config.
         */
        pthread_mutex_t access_;

        // private attributes and methods
        eskf_odometry filter_;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * \brief define config type
         *
         * Define a Config type with the EskfOdomConfig. All driver implementations
         * will then use the same variable type Config.
         */
        typedef eskf_odom::EskfOdomConfig Config;

        /**
         * \brief config variable
         *
         * This variable has all the driver parameters defined in the cfg config file.
         * Is updated everytime function config_update() is called.
         */
        Config config_;

        /**
         * \brief constructor
         *
         * In this constructor parameters related to the specific driver can be
         * initalized. Those parameters can be also set in the openDriver() function.
         * Attributes from the main node driver class IriBaseDriver such as loop_rate,
         * may be also overload here.
         */
        EskfOdomAlgorithm(void);

        /**
         * \brief Lock Algorithm
         *
         * Locks access to the Algorithm class
         */
        void lock(void) { pthread_mutex_lock(&this->access_); };

        /**
         * \brief Unlock Algorithm
         *
         * Unlocks access to the Algorithm class
         */
        void unlock(void) { pthread_mutex_unlock(&this->access_); };

        /**
         * \brief Tries Access to Algorithm
         *
         * Tries access to Algorithm
         *
         * \return true if the lock was adquired, false otherwise
         */
        bool try_enter(void)
        {
            if(pthread_mutex_trylock(&this->access_)==0)
                return true;
            else
                return false;
        };

        /**
         * \brief config update
         *
         * In this function the driver parameters must be updated with the input
         * config variable. Then the new configuration state will be stored in the
         * Config attribute.
         *
         * \param new_cfg the new driver configuration state
         *
         * \param level level in which the update is taken place
         */
        void config_update(Config& new_cfg, uint32_t level=0);

        /**
         * \brief Set filter initial parameters
         *
         * This function calls the low-level library to set the initial values of the filter, imu, px4,
         * nominal-state vector and error-state vector. Wrapper of the low-level library
         *
         */
        // void set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu,const Sensor::pose_params& pose,const Sensor::pose_params& pose2,const Sensor::position_params& position,const Sensor::orientation_params& orientation,const Sensor::linvel_params& linvel,const Sensor::range_params& range,const Sensor::px4_params& px4,const Sensor::flow2d_params& flow2d);

        /**
         * \brief Set filter initial parameters
         *
         * This function calls the low-level library to set the initial values of the filter, imu, px4,
         * nominal-state vector and error-state vector. Wrapper of the low-level library
         *
         */
        void set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu,const Sensor::pose_params& pose,const Sensor::pose_params& pose2,const Sensor::position_params& position,const Sensor::orientation_params& orientation,const Sensor::linvel_params& linvel);


        /**
         * \brief Print initial parameters
         *
         * This function is used to print the parameters set of the filter, IMU, PX4
         * and nominal-state and error-state vectors. Wrapper of the low-level library
         *
         */
        void print_ini_params(void);

        /**
         * \brief Get processing time.
         *
         * Get filter current processing time.
         *
         */
        float get_proc_time(void);

        /**
         * \brief Set Observation phase
         *
         * Sets the observation values depending on the robot phase.
         * This step is required due to px4 limited range (0.3m<pz<5m)
         * Different values for the sensor:
         *   -LANDED: sinthetic values (robot not moving).
         *   -TOFF,LANDING: High covariance in order to reduce update stage
         *   -FLYING: sensor readings and corresponding covariances.
         *
         * Inputs:
         *   There are no specific inputs due to all are class variables.
         */
        void set_obs_phase(const bool& flying, const float& gnd_dist);

        /**
         * \brief Get imu parameters
         *
         * Get imu parameters
         *
         */
        // Sensor::imu_params get_imu_params(void);

        /**
         * \brief Set imu parameters
         *
         * Set imu parameters
         *
         */
        // void set_imu_params(Sensor::imu_params& imu);

        /**
         * \brief Set IMU Readings
         *
         * Store new IMU readings
         *
         * Input:
         *   t: Time stamp.
         *   a: Acc. readings (m/s^2). a = [ax,ay,az].
         *   w: Gyro. readings (rad/s). w = [wx,wy,wz].
         *   q: Orientation [OPTIONAL]. q = [qw,qx,qy,qz].
         */
        void set_imu_reading(const float& dt, const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::Quaternionf& nwu_q_imu);
        void set_imu_reading(const float& t, const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::Quaternionf& q, const Eigen::Quaternionf& nwu_q_imu);

        /**
         * \brief Get pose parameters
         *
         * Get pose parameters
         *
         */
        // Sensor::pose_params get_pose_params(void);

        /**
         * \brief Get pose 2 parameters
         *
         * Get pose 2 parameters
         *
         */
        // Sensor::pose_params get_pose2_params(void);

        /**
         * \brief Set pose parameters
         *
         * Set pose parameters
         *
         */
        // void set_pose_params(Sensor::pose_params& pose);

        /**
         * \brief Set pose 2 parameters
         *
         * Set pose 2 parameters
         *
         */
        // void set_pose2_params(Sensor::pose_params& pose);

        /**
         * \brief Set Pose Readings
         *
         * Store new Pose readings
         *
         * Input:
         *   t: Time stamp.
         *   val: Pose reading = [p_x,p_y,p_z,r_x,r_y,r_z].
         */
        void set_pose_reading(const float& t, const Eigen::VectorXf& val);

        /**
         * \brief Set Pose 2 Readings
         *
         * Store new Pose 2 readings
         *
         * Input:
         *   t: Time stamp.
         *   val: Pose 2 reading = [p_x,p_y,p_z,r_x,r_y,r_z].
         */
        void set_pose2_reading(const float& t, const Eigen::VectorXf& val);

        /**
         * \brief Get position parameters
         *
         * Get position parameters
         *
         */
        // Sensor::position_params get_position_params(void);

        /**
         * \brief Set position parameters
         *
         * Set position parameters
         *
         */
        // void set_position_params(Sensor::position_params& position);

        /**
         * \brief Set Position Readings
         *
         * Store new Position readings
         *
         * Input:
         *   t: Time stamp.
         *   val: Position reading = [p_x,p_y,p_z].
         */
        void set_position_reading(const float& t, const Eigen::Vector3f& val);

        /**
         * \brief Get orientation parameters
         *
         * Get orientation parameters
         *
         */
        // Sensor::orientation_params get_orientation_params(void);

        /**
         * \brief Set orientation parameters
         *
         * Set orientation parameters
         *
         */
        // void set_orientation_params(Sensor::orientation_params& orientation);

        /**
         * \brief Set Pose Readings
         *
         * Store new Pose readings
         *
         * Input:
         *   t: Time stamp.
         *   val: Orientation reading = [r_x,r_y,r_z].
         */
        void set_orientation_reading(const float& t, const Eigen::Vector3f& val);

        /**
         * \brief Get linear velocity parameters
         *
         * Get linear velocity parameters
         *
         */
        // Sensor::linvel_params get_linvel_params(void);

        /**
         * \brief Set linear velocity parameters
         *
         * Set linear velocity parameters
         *
         */
        // void set_linvel_params(Sensor::linvel_params& linvel);

        /**
         * \brief Set Linear Velocity Readings
         *
         * Store new Linear Velocity readings
         *
         * Input:
         *   t: Time stamp.
         *   val: Linear velocity reading = [v_x,v_y,v_z].
         */
        void set_linvel_reading(const float& t, const Eigen::Vector3f& val);

        /**
         * \brief Get range parameters
         *
         * Get range parameters
         *
         */
        // Sensor::range_params get_range_params(void);

        /**
         * \brief Set range parameters
         *
         * Set range parameters
         *
         */
        // void set_range_params(Sensor::range_params& range);

        /**
         * \brief Set Range Readings
         *
         * Store new Range readings
         *
         * Input:
         *   t: Time stamp.
         *   val: Range reading.
         */
        void set_range_reading(const float& t, const Eigen::VectorXf& val);

        /**
         * \brief Get px4 parameters
         *
         * Get px4 parameters
         *
         */
        // Sensor::px4_params get_px4_params(void);

        /**
         * \brief Set px4 parameters
         *
         * Set px4 parameters
         *
         */
        // void set_px4_params(Sensor::px4_params& px4);

        /**
         * \brief Set PX4 Optical Flow Readings
         *
         * Store new PX4 Optical Flow readings
         *
         * Input:
         *   t: Time stamp.
         *   data: Sensor readings: val = [pz;vx;vy;flow_x;flow_y].
         */
        void set_px4_reading(const float& t, const Eigen::VectorXf& data);

        /**
         * \brief Get flow2d parameters
         *
         * Get flow2d parameters
         *
         */
        // Sensor::flow2d_params get_flow2d_params(void);

        /**
         * \brief Set flow2d parameters
         *
         * Set flow2d parameters
         *
         */
        // void set_flow2d_params(Sensor::flow2d_params& flow2d);

        /**
         * \brief Set Flow2d Readings
         *
         * Store new Flow2d readings
         *
         * Input:
         *   t: Time stamp.
         *   val: Optical flow raw reading flow = [flow_x,flow_y].
         */
        void set_flow2d_reading(const float& t, const Eigen::Vector2f& val);

        /**
         * \brief Run next step
         *
         * Run filter next step (either propagation or update and correction)
         *
         * Input/Ouput:
         *   state:  Robot state (6D).
         *   ang_vel: Robot Angular Velocity (currently not included in the state).
         */
        bool update(Eigen::VectorXf& state, Eigen::Vector3f& ang_vel, const bool& flying, const float& gnd_dist);

        void update(Eigen::VectorXf& state,Eigen::VectorXf& covPtrue);

        /**
         * \brief Destructor
         *
         * This destructor is called when the object is about to be destroyed.
         *
         */
        ~EskfOdomAlgorithm(void);
};

#endif
