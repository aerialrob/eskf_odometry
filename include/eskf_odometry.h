#ifndef ESKF_ODOMETRY_H
#define ESKF_ODOMETRY_H

// Eigen
#include <Eigen/Dense>

// Get the floating point relative accurancy
// float EPS_ = std::nextafterf(0.0, 1);s


struct params {
    Eigen::VectorXf dp0_std ;
    Eigen::VectorXf dv0_std;
    Eigen::VectorXf dtheta0_std;
    Eigen::VectorXf dab0_std;
    Eigen::VectorXf dwb0_std;
    Eigen::VectorXf dg0_std;
    Eigen::VectorXf dpo0_std;
    Eigen::VectorXf dthetao0_std;
    Eigen::VectorXf dpr0_std;
    Eigen::VectorXf dthetar0_std;
    char frame;
} ;


class Sensor
{
public:
    struct imu_params {
        Eigen::VectorXf std = Eigen::VectorXf(12);        
        char met;
    } ;

    struct position_params {
        Eigen::VectorXf std_outsidebounds;        
        Eigen::VectorXf std_insidebounds;      
        Eigen::VectorXf std;      
    } ;

    struct orientation_params {
        Eigen::VectorXf std_outsidebounds;        
        Eigen::VectorXf std_insidebounds;      
        Eigen::VectorXf std;      
    } ;

    struct linvel_params {
        Eigen::VectorXf std_outsidebounds;        
        Eigen::VectorXf std_insidebounds;      
        Eigen::VectorXf std;      
    } ;

    struct pose_params {
        Eigen::VectorXf std_outsidebounds;        
        Eigen::VectorXf std_insidebounds;      
        Eigen::VectorXf std;      
    } ;

    struct range_params {
        Eigen::VectorXf std_outsidebounds;        
        Eigen::VectorXf std_insidebounds;      
        Eigen::VectorXf std;      
        float range_min;
        float range_max;
        bool using_laser;
    } ;    
};


class eskf_odometry
{
public:
        params f_params_;
        Eigen::VectorXf external_state = Eigen::VectorXf(19);            // dejar en 19 posiciones
        Eigen::VectorXf internal_state = Eigen::VectorXf(18);            // rpy seg(6:3)
        Eigen::MatrixXf Ptrue = Eigen::MatrixXf::Identity(18,18);       
        Eigen::VectorXf vPtrue_ = Eigen::VectorXf(18);  
        Eigen::VectorXf vPtrue = Eigen::VectorXf(19);   // quaternion seg(6:4)
        Eigen::MatrixXf Qi = Eigen::MatrixXf::Identity(12,12);  
        Eigen::MatrixXf Fi = Eigen::MatrixXf::Zero(18,12);
        Eigen::MatrixXf FiQi = Eigen::MatrixXf::Zero(18,18);

        Eigen::Vector3f a_ = Eigen::VectorXf::Zero(3);
        Eigen::Vector3f w_ = Eigen::VectorXf::Zero(3);

        Eigen::MatrixXf Ra_ = Eigen::MatrixXf::Zero(3,3);
        Eigen::MatrixXf Rw_ = Eigen::MatrixXf::Zero(3,3);


        // Definicion a y w para guardar directamente última lectura de la imu 


        Eigen::VectorXf dx0_;
        // Init cov matrice
        Sensor::imu_params imu_;
        Sensor::pose_params pose_; 
        Sensor::pose_params pose2_;
        Sensor::position_params position_;
        Sensor::orientation_params orientation_;
        Sensor::linvel_params linvel_;

        float t_filter;
        float t_filter_prev;

        bool is_first_imu_received = false;

        /**
         * \brief constructor
         *
         * In this constructor parameters related to the specific driver can be
         * initalized. Those parameters can be also set in the openDriver() function.
         * Attributes from the main node driver class IriBaseDriver such as loop_rate,
         * may be also overload here.
         */
        eskf_odometry();
        
        /**
         * \brief Set filter initial parameters
         *
         * This function calls the low-level library to set the initial values of the filter, imu, px4,
         * nominal-state vector and error-state vector. Wrapper of the low-level library
         *
         */
        void set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu, const Sensor::position_params& position, const Sensor::orientation_params& orientation);
        
        /**
         * \brief Print initial parameters
         *
         * This function is used to print the parameters set of the filter, IMU, PX4
         * and nominal-state and error-state vectors. Wrapper of the low-level library
         *
         */
        void print_ini_params();

        /**
         * \brief set_pose_reading       
         *
         * Store new pose readings
         
        Input:
            - t: Time stamp of the message
            - msg: Pose reading = [p_x,p_y,p_z]
        Output            
            -1: Message not processed because first imu msg not received
            0 : Message too old to process
            1 : Message successfully processed                 
        */
        // int set_pose_reading(const float& t, const Eigen::VectorXf& msg);
        // int set_pose_reading(const float& t_msg, const Eigen::VectorXf& msg, const Eigen::VectorXf& cov);
        
        int set_position_reading(const float& t_msg, const Eigen::VectorXf& msg, const Eigen::MatrixXf& R);

        /**
         * \brief set_pose_reading       
         *
         * Store new pose readings
         
        Input:
            - t: Time stamp of the message
            - msg: Mag reading = [p_x,p_y,p_z]
        Output            
            -1: Message not processed because first imu msg not received
            0 : Message too old to process
            1 : Message successfully processed                 
        */
        int set_magnetometer_reading(const float& t_msg, const Eigen::VectorXf& msg, const Eigen::MatrixXf& R);
       
        /**
         * \brief propagate_imu
         * 
         * Mean State Prediction and Covariance Prediction from imu message
         
        Input:
            - t: Time stamp of the message
            - a: Acc. readings (m/s^2). a = [ax,ay,az].
            - w: Gyro. readings (rad/s). w = [wx,wy,wz].
        Output            
            0 : Message too old to process
            1 : Message successfully processed              
        */
        int set_imu_reading(const float& t_msg, const Eigen::Vector3f& a, const Eigen::Vector3f& w);     
        int set_imu_reading(const float& t_msg, const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::MatrixXf& Ra, const Eigen::MatrixXf& Rw);
        






        //----------------- Mean State Prediction and Covariance Prediction ESKF   error-state ---------------------
        void propagate_state();   
        
        /**
         * \brief MEAN STATE PREDICTION       
         *
         * Returns the prediction step for the nominal-state vector.
         
        Input:
            - xstate:   Nominal-state vector at time k-1.
            - a_s:      Acc. reading at time k.
            - w_s:      Gyro readings at time k.
            - dt:       Time step between k-1 and k.dt: Time stamp difference step between k and k-1        
        Outputs:
            - xstate_nom:   Nominal-state vector estimate at time k.   
        */
        Eigen::VectorXf mean_predict(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt);
        
        /**
         * \brief q mean state prediction
         *
         * Returns the q prediction step for the nominal-state vector.
        Inputs:
            - q:    Quaternion.
            - w:    Rotation in body frame or Rotation speed expressed 
                      by the three angles (roll, pitch, yaw) or three angle rates.
            - dt:   Time step..
        Outputs:
            - q1:   Quaternion after a rotation in body frame.
        */
        Eigen::VectorXf qPredict(const Eigen::VectorXf& q, const Eigen::Vector3f& w, const float& dt);

        /**
         * \brief Covariance Prediction
         *
         * Returns the Covariance Prediction for the nominal-state vector.         
        Inputs:
            - P_old:        Covariance matrix at time k-1.
            - Fx:           ESKF Transition matrix.
            - Fi:           Jacobian that maps the IMU covariance.
            - Qi:           Imu covariance matrix.
        Outputs:
            - q1:           Quaternion after a rotation in body frame.
        */
        Eigen::MatrixXf cov_predict(const Eigen::VectorXf& P_old, const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt);

        /**
         * \brief innovation     
         *
         * Computes innovation z and innovation's covariances matrix Z 
         
        Input:
            - y:        Measurement.
            - e:        Expectation.
            - P_old:    Covariance matrix at time k-1.
            - H:        Observation Jacobian.
            - Q:        Observation covariance matrix.         
        Outputs:
            - dz:       Residual or innovation.
            - Z:        Residual or innovation covariance matrix.
        */
        void innovation(const Eigen::VectorXf& y, const Eigen::VectorXf& e, const Eigen::MatrixXf& P_old, const Eigen::MatrixXf& H, const Eigen::MatrixXf& Q,
                Eigen::VectorXf& dz, Eigen::MatrixXf& Z);

        /**
         * \brief correct_state     
         *
         * Calculate posterior state vector and covariance matrix
         
        Input:
            - y:        Measurement.
            - y_hat:    Expectation.            
            - H:        Observation Jacobian.
            - R:        Observation covariance matrix.         
        */
        void correct_state(const Eigen::VectorXf&y, const Eigen::VectorXf&y_hat, const Eigen::MatrixXf& H, const Eigen::MatrixXf& R);


        void update(Eigen::VectorXf& state, Eigen::VectorXf& covPtrue);

        void reset_state();
};






#endif // ESKF_ODOMETRY_H
