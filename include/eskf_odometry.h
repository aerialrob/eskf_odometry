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
        Eigen::VectorXf xtrue = Eigen::VectorXf(33);            // dejar en 19 posiciones
        Eigen::MatrixXf Ptrue = Eigen::MatrixXf::Identity(18,18);       
        Eigen::VectorXf vPtrue_ = Eigen::VectorXf(18); 
        Eigen::VectorXf vPtrue = Eigen::VectorXf(19);
        Eigen::MatrixXf Qi = Eigen::MatrixXf::Identity(12,12);
        Eigen::MatrixXf Fi = Eigen::MatrixXf::Zero(18,12);
        Eigen::MatrixXf FiQi = Eigen::MatrixXf::Zero(18,18);

        // Definicion a y w para guardar directamente última lectura de la imu 


        Eigen::VectorXf dx0_;
        // Init cov matrice
        Sensor::imu_params imu_;
        Sensor::pose_params pose_; 
        Sensor::pose_params pose2_;
        Sensor::position_params position_;
        Sensor::orientation_params orientation_;
        Sensor::linvel_params linvel_;

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
        void set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu,const Sensor::pose_params& pose,const Sensor::pose_params& pose2,const Sensor::position_params& position,const Sensor::orientation_params& orientation,const Sensor::linvel_params& linvel);
        /**
         * \brief Print initial parameters
         *
         * This function is used to print the parameters set of the filter, IMU, PX4
         * and nominal-state and error-state vectors. Wrapper of the low-level library
         *
         */
        void print_ini_params();
        /**
         * \brief propagate_imu
         *
         * Mean State Prediction and Covariance Prediction ESKF   error-state
         *
         * Input:
         *   dt: Time stamp.
         *   a: Acc. readings (m/s^2). a = [ax,ay,az].
         *   w: Gyro. readings (rad/s). w = [wx,wy,wz].
         */
        void propagate_imu(const float& dt_imu, const Eigen::Vector3f& a, const Eigen::Vector3f& w);

        /**
         * \brief set_pose_reading                  
         * Store new pose readings
         *
         * Input:
         *   t: Time stamp.
         *   val: pos x,y,z and q    
         */
        void set_pose_reading(const float& t, const Eigen::VectorXf& val);
        /**
         * \brief MEAN STATE PREDICTION
         *
         * Returns the prediction step for the nominal-state vector.
         *
         * Input:
         *   xstate:   Nominal-state vector at time k-1.
         *   a_s:      Acc. reading at time k.
         *   w_s:      3x2 matrix with Gyro readings (columns) at time k-1 and k.
         *   dt:       Time step between k-1 and k.
         * Outputs:
         *   xstate_nom:   Nominal-state vector estimate at time k.
         */
        Eigen::VectorXf mean_predict(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt);
        /**
         * \brief q mean state prediction
         *
         * Returns the q prediction step for the nominal-state vector.
         *
         *   Inputs:
         *      - q:    Quaternion.
         *      - w:    Rotation in body frame or Rotation speed expressed 
         *              by the three angles (roll, pitch, yaw) or three angle rates.
         *      - dt:   Time step..
         * Outputs:
         *      - q1:   Quaternion after a rotation in body frame.
         */
        Eigen::VectorXf qPredict(const Eigen::VectorXf& q, const Eigen::Vector3f& w, const float& dt);

        Eigen::MatrixXf cov_predict(const Eigen::VectorXf& P_old, const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt);

        void update(Eigen::VectorXf& state, Eigen::VectorXf& covPtrue);
};






#endif // ESKF_ODOMETRY_H
