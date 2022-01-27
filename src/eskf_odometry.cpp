#include "eskf_odometry.h"


// STD stuff
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>      

// IMU transition matrix from error state (ESKF) 
Eigen::MatrixXf imu_trans_mat(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt)
{    
    // Derivative r.t. Error-State _____________________________________________
    Eigen::VectorXf q_ = xstate.segment(6,4);
    Eigen::VectorXf ab = xstate.segment(10,3);
    Eigen::VectorXf wb = xstate.segment(13,3);
    Eigen::Quaternionf q{q_(0), q_(1), q_(2), q_(3)};
    Eigen::Matrix3f R;
    atools::q2R(q, R);         

    // Product on the left hand side of the nominal quaternion
    // Global Error (GE)
    Eigen::Matrix3f M_sk(3,3);    
    Eigen::Vector3f vsk_Ra = R*(a_s-ab);
    atools::v2skew(vsk_Ra, M_sk);
    Eigen::Matrix3f V = -M_sk; //3x3 * 1x3    


    Eigen::MatrixXf F_dxstate(18,18);

    Eigen::MatrixXf M0 = Eigen::MatrixXf::Zero(3,3);
    Eigen::MatrixXf MIdent = Eigen::MatrixXf::Identity(3,3);
    float t1 = dt;
    float t2 = 0.5*(dt*dt);
    float t3 = (dt*dt*dt)/6;

    F_dxstate << MIdent,    MIdent*t1, V*t2,   -R*t2,     -V*R*t3,  MIdent*t2,
                 M0,        MIdent,    V*t1,   -R*t1,     -V*R*t2,  MIdent*t1,
                 M0,        M0,        MIdent, M0,        -R*t1,    M0,
                 M0,        M0,        M0,     MIdent,    M0,       M0,
                 M0,        M0,        M0,     M0,        MIdent,   M0,
                 M0,        M0,        M0,     M0,        M0,       MIdent;      

    return F_dxstate;
}




// ---------------------------------------------------- Class methods
eskf_odometry::eskf_odometry()
{

}

void eskf_odometry::set_init_params(const params& f_params, const Eigen::VectorXf& xtrue0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu, const Sensor::position_params& position, const Sensor::orientation_params& orientation, const Sensor::gravity_params& gravity, const Sensor::magnetometer_params& magnetometer)
{
    std::cout << "Low level init " << std::endl;
    // Initial State
    f_params_ = f_params;
    external_state = xtrue0;
    internal_state = dx0;

    // Std from params
    imu_params_ = imu;
    position_ = position;
    orientation_ = orientation; 
    gravity_params_ = gravity;  
    mag_params_ =  magnetometer;

    // Std from param server
    // Ra_ = imu_params_.std.segment(0,3).asDiagonal();
    // Rw_ = imu_params_.std.segment(3,3).asDiagonal();

    Rba_ = imu_params_.std.segment(6,3).asDiagonal();
    Rbw_ = imu_params_.std.segment(9,3).asDiagonal();
    Rg_ = gravity_params_.std.asDiagonal();
     

    // Initial Covariance
    Eigen::VectorXf std_ini = Eigen::VectorXf(18);   

    std_ini << f_params_.dp0_std, f_params_.dv0_std, f_params_.dtheta0_std, f_params_.dab0_std, f_params_.dwb0_std, f_params_.dg0_std;          
    std_ini = std_ini.array() * std_ini.array();

    Eigen::MatrixXf Ptrue0;
    Ptrue0 = std_ini.asDiagonal();

    Ptrue = Ptrue0;

}

void eskf_odometry::print_ini_params()
{
    std::cout << "********Print level init*********" << std::endl;
    std::cout << "Initial external-State vector (p,v,q,ab,wb,g) size " << external_state.size() << std::endl;
    std::cout << "external_state" << std::endl;
    std::cout << external_state << std::endl;
    std::cout << "# Initial Error-State vector (dp,dv,dtheta,dab,dwb,dg) size " << internal_state.size() << std::endl;
    std::cout << "internal_state" << std::endl;
    std::cout << internal_state << std::endl;
    std::cout << "# System Covariance matrix P size " << internal_state.size() << std::endl;
    std::cout << "System Covariance matrix P " << std::endl;
    std::cout << Ptrue << std::endl;

    std::cout << "# Sensors Parameter Covariance " << internal_state.size() << std::endl;
    // std::cout << "Ra_" << std::endl;
    // std::cout << Ra_ << std::endl;
    // std::cout << "Rw_" << std::endl;
    // std::cout << Rw_ << std::endl;    
    std::cout << "Rba_" << std::endl;
    std::cout << Rba_ << std::endl;
    std::cout << "Rbw_" << std::endl;
    std::cout << Rbw_ << std::endl;
    std::cout << "Rg_" << std::endl;
    std::cout << Rg_ << std::endl;
    std::cout << "mag_params_vector" << std::endl;
    std::cout << mag_params_.magnetic_field_vector[0] << std::endl;
    std::cout << mag_params_.magnetic_field_vector[1] << std::endl;
    std::cout << mag_params_.magnetic_field_vector[2] << std::endl;
    std::cout << std::endl << "********END Print level init*********" << std::endl;

    atools::print(Ptrue, blue);
}

int eskf_odometry::set_imu_reading(const float& t_msg, const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::MatrixXf& Ra, const Eigen::MatrixXf& Rw)
{
    // std::cout << std::endl << "IMU reading Low level " << std::endl;
    // std::cout << std::fixed << std::setprecision(10) << t_msg << " t_msg IMU " << std::endl;
    // Save imu a, w and cov values
    a_ = a;
    w_ = w;
    Ra_ = Ra;
    Rw_ = Rw;
    
    if (!is_first_imu_received) // not first imu call, initialize t_filter
    {
        this->t_filter_prev = t_msg;
        this->t_filter = t_msg;
        is_first_imu_received = true;
        // std::cout << std::fixed << std::setprecision(10) << t_filter_prev << " t_filter_prev " << std::endl;
        // std::cout << std::fixed << std::setprecision(10) << t_filter << " t_filter " << std::endl;
        std::cout << "FIRST IMU " << t_filter << std::endl;
        return 1;        
    }    
    if (t_msg < t_filter )
        return 0;

    // Process sensor   
    this->t_filter_prev = t_filter;
    this->t_filter = t_msg;
    // std::cout << std::fixed << std::setprecision(10) << t_filter_prev << " t_filter_prev " << std::endl;
    // std::cout << std::fixed << std::setprecision(10) << t_filter << " t_filter " << std::endl;
    
    // atools::print("-- IMU Propagate --", magenta); std::cout << std::endl;
    propagate_state();    
    return 1;
}

//----------------- Mean State Prediction and Covariance Prediction ESKF   error-state ---------------------
void eskf_odometry::propagate_state()  
{
    // dt
    float dt_filter = t_filter - t_filter_prev;

    // Error initialized to 0 at every step
    // internal_state = Eigen::VectorXf::Zero(18);            // rpy seg(6:3)

    // Prediction __________________________________________________           
    mean_predict(a_, w_, dt_filter, external_state);
    // atools::print("---state pred---", cyan);  std::cout << std::endl;
    // atools::print(external_state, cyan);  std::cout << std::endl;

    // Covariance Prediction ESKF   error-state       
    cov_predict(external_state, a_, w_, dt_filter, Ptrue);
    vPtrue_ = Ptrue.diagonal();
    Eigen::Vector3f qvector = vPtrue_.segment(6,3);
    Eigen::Quaternionf q;
    atools::v2q(qvector, q); 
    q = q.normalized();   
    Eigen::Vector4f q1_norm(q.w(), q.x(), q.y(), q.z());

    // Cov vector 
    vPtrue << vPtrue_.segment(0,3), vPtrue_.segment(3,3), q1_norm, vPtrue_.segment(9,3), vPtrue_.segment(12,3), vPtrue_.segment(15,3);
    // atools::print("---Cov Pred---", red);  std::cout << std::endl;
    // atools::print(vPtrue, red);  std::cout << std::endl;    

}


//----------------- Mean State Prediction ---------------------
void eskf_odometry::mean_predict(const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt, Eigen::VectorXf& xstate)
{
    // Propagate Bias and gravity (considering linear during integration)
    Eigen::VectorXf p = xstate.segment(0,3);
    Eigen::VectorXf v = xstate.segment(3,3);
    Eigen::VectorXf q = xstate.segment(6,4);
    Eigen::VectorXf ab = xstate.segment(10,3);
    Eigen::VectorXf wb = xstate.segment(13,3);
    Eigen::VectorXf g = xstate.segment(16,3);

    // Angular velocity in Inertial frame (= body Frame, w/o bias)
    Eigen::Vector3f w = w_s - wb;       //eq (22c)
    
    Eigen::Quaternionf quat{xstate[6], xstate[7], xstate[8], xstate[9]};        
    Eigen::Quaternionf qpred;
    //----------------- q prediction step for the nominal-state vector.------------------
    atools::qPredict(quat, w, qpred, dt);   //Exact method qpred normalized    
    // atools::qPredict(quat, w, qpred);   //Exact method qpred normalized    
    atools::print("--q prediction--", white);  std::cout << std::endl;
    atools::print(qpred, white);  std::cout << std::endl;

    // Acceleration in Inertial frame (w/o bias and g). This has to be coherent
    // with Quaternionn integration method. The time step measure must be the same 
    // at k-1 or k
    // Method zero-th backward
    Eigen::Vector3f a;    
    Eigen::Matrix3f R;
    atools::q2R(qpred, R);  
    a = R*(a_s - ab) + g; // 3x3 * 1x3 = 3*1
    // std::cout << "a " << a << std::endl;
    // Position and Velocity
    Eigen::Vector3f p_nom;
    Eigen::Vector3f v_nom;    

    int trunc_met = 1;  // Truncated method 
    p_nom = p + v*dt; // Linear position in Inertial frame (integrating vel. and acc.)
    v_nom = v + a*dt; // Linear velocity in Inertial frame (integrating acc.)
    // Return vector
    // xstate_nom = [p_nom;v_nom;q_nom;ab;wb;g]; % Nominal state
    Eigen::Vector4f q_nom; 
    q_nom << qpred.w(), qpred.x(), qpred.y(), qpred.z();
    // Eigen::VectorXf xstate_nom(xstate.size());
    // xstate_nom << p_nom, v_nom, q_nom, ab, wb, g;
    xstate << p_nom, v_nom, q_nom, ab, wb, g;

    // Componer msg ROS
    // xstate << xstate_nom;    
    // return xstate_nom;    
}


//-----------------Imu Covariance Error Propagation  ESKF Transition matrix.---------------------
void eskf_odometry::cov_predict(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt, Eigen::MatrixXf& P_old)
{
    // Covariance Error Propagation
    // IMU transition matrix from error state (ESKF) 
    Eigen::MatrixXf F_dxstate(18,18);
    // F_dxstate:           ESKF Transition matrix.
    F_dxstate = imu_trans_mat(xstate, a_s, w_s, dt);  

        FiQi << Eigen::MatrixXf::Zero(3,18),
        Eigen::MatrixXf::Zero(3,3), Ra_, Eigen::MatrixXf::Zero(3,12), 
        Eigen::MatrixXf::Zero(3,6), Rw_, Eigen::MatrixXf::Zero(3,9),
        Eigen::MatrixXf::Zero(3,9), Rba_, Eigen::MatrixXf::Zero(3,6),
        Eigen::MatrixXf::Zero(3,12), Rbw_, Eigen::MatrixXf::Zero(3,3), 
        Eigen::MatrixXf::Zero(3,15), Rg_; 
        // std::cout << "FiQi size" << std::endl;
        // std::cout << FiQi.size() << std::endl;
        // std::cout << FiQi << std::endl;


    P_old = F_dxstate * P_old * F_dxstate.transpose() + FiQi; // 18*18    transpose()
    P_old = (P_old + P_old.transpose()) * 0.5;    //force symmetric

}

//-----------------Position.---------------------
int eskf_odometry::set_position_reading(const float& t_msg, const Eigen::Vector3f& msg, const Eigen::Matrix3f& R)
{
    if(!is_first_imu_received)   // not first imu call, return
        return -1;
    if (t_msg < t_filter )  // msg time in the past, return
        return 0;

    // Process sensor   
    t_filter_prev = t_filter;
    t_filter = t_msg;
    // std::cout << std::fixed << std::setprecision(10) << t_filter_prev << " t_filter_prev " << std::endl;
    // std::cout << std::fixed << std::setprecision(10) << t_filter << " t_filter " << std::endl;
    // std::cout << "--            POS Propagate --" << std::endl;
    atools::print("-- POS Propagate --", cyan); std::cout << std::endl;
    propagate_state();
    // Evaluate mesurement function
    Eigen::VectorXf y_hat = external_state.segment(0,3); // Pos x, y, z from external_state
    // Evaluate mesurement jacobian      
    Eigen::MatrixXf H(3,18);
    H << Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,15);   //(3x18)  
           
    // std::cout << "--            POS Correct --" << std::endl;
    atools::print("-- POS Correct --", cyan); std::cout << std::endl;
    correct_state(msg, y_hat, H, R); 
    atools::print("-- POS Reset --", cyan); std::cout << std::endl;
    // std::cout << "--            POS Reset --" << std::endl;
    reset_state();

    return 1;
}

int eskf_odometry::set_magnetometer_reading(const float& t_msg, const Eigen::VectorXf& msg, const Eigen::MatrixXf& R)
{
    if(!is_first_imu_received)   // not first imu call, return
        return -1;
    if (t_msg < t_filter )  // msg time in the past, return
        return 0;

    // Process sensor   
    t_filter_prev = t_filter;
    t_filter = t_msg;
  
    atools::print("-- MAG Propagate --", green); std::cout << std::endl;
    propagate_state();
    // Evaluate mesurement function
    Eigen::Quaternionf qt{external_state[6], external_state[7], external_state[8], external_state[9]};
    Eigen::Matrix3f R_temp;

    Eigen::Vector3f magnetic_field_vector{mag_params_.magnetic_field_vector[0], mag_params_.magnetic_field_vector[1], mag_params_.magnetic_field_vector[2]};   

    // atools::q2R(q,Rot);
    R_temp = qt.matrix();
    // Eigen::Vector3f y_hat = Rot.transpose() * magnetic_field_vector;
        // Magnetic field msg 
    Eigen::Vector3f y_hat;    
    y_hat = R_temp.transpose() * magnetic_field_vector; 
    
    // Evaluate mesurement jacobian      
    Eigen::MatrixXf H(msg.size(),18);       
    Eigen::Matrix3f M_sk(3,3);
    atools::v2skew(magnetic_field_vector, M_sk);

    H << Eigen::MatrixXf::Zero(3,6),  R_temp.transpose() * M_sk, Eigen::MatrixXf::Zero(3,9);
    // H << Eigen::MatrixXf::Zero(3,6),  Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,9);
             
    atools::print("-- MAG Correct --", green); std::cout << std::endl;
    atools::print("H-------", green); std::cout << std::endl;
    atools::print(H, green); std::cout << std::endl;
    atools::print("R-------", green); std::cout << std::endl;
    atools::print(R, green); std::cout << std::endl;
    correct_state(msg, y_hat, H, R); 
    atools::print("-- MAG Reset --", green); std::cout << std::endl;
    reset_state();

    return 1;
}

Eigen::Quaternionf eskf_odometry::quatFromHamilton(const Eigen::Vector4f& qHam) {
    return Eigen::Quaternionf(
        (Eigen::Vector4f() <<
            qHam.block<3, 1>(1, 0), // x, y, z
            qHam.block<1, 1>(0, 0) // w
        ).finished());
}

Eigen::Vector3f eskf_odometry::quatToRotVec(const Eigen::Quaternionf& q) {
    Eigen::AngleAxisf angAx(q);
    return angAx.angle() * angAx.axis();
}


int eskf_odometry::set_orientation_reading(const float& t_msg, const Eigen::Quaternionf& q_gb_meas, const Eigen::Matrix3f& theta_covariance)
{

}

void eskf_odometry::update_3D(
        const Eigen::Vector3f& delta_measurement,
        const Eigen::Matrix3f& meas_covariance,
        const Eigen::Matrix<float, 3, dSTATE_SIZE>& H) {

    // Kalman gain
    Eigen::Matrix<float, dSTATE_SIZE, 3> PHt = Ptrue*H.transpose();
    Eigen::Matrix<float, dSTATE_SIZE, 3> K;
    // if((delayHandling_ == noMethod || delayHandling_ == applyUpdateToNew || delayHandling_ == larsonAverageIMU)){
           K = PHt * (H*PHt + meas_covariance).inverse();
    // }
    // if(delayHandling_ == larsonAverageIMU && !normalPass){
        // K = F_x_*K;
    // }
    // Correction error state
    // Matrix<float, dSTATE_SIZE, 1> errorState = K * delta_measurement;
    internal_state = K * delta_measurement;
    // Update P (simple form)
    // P = (I_dx - K*H)*P;
    // Update P (Joseph form)
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I_KH = I_dx - K*H;
    // if(delayHandling_ == noMethod || delayHandling_ == applyUpdateToNew){
        Ptrue = I_KH*Ptrue*I_KH.transpose() + K*meas_covariance*K.transpose();
    // }
    // if(delayHandling_ == larsonAverageIMU  && !normalPass){

    //     P_ = P_ - K*H*PHistoryPtr_->at(bestTimeIndex).second*F_x_;
    // }

    // injectErrorState(errorState);
}

void eskf_odometry::correct_state(const Eigen::Vector3f& delta_measurement, const Eigen::MatrixXf& H, const Eigen::MatrixXf& R)
{
    // Innovation   
    Eigen::Vector3f dz  = delta_measurement;    //1x3                                  //1x6

    //  Observation covariance matrix
    Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(3, 3);
    Z = H * Ptrue * H.transpose() + R;   //3x18 * 18x18 * 18x3 + 3x3            //6x18 * 18x18 * 18x6 + 6x6 = 6x6
    Z = (Z + Z.transpose())*0.5;

    // Kalman Gain
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(18, 3);
    K = Ptrue * H.transpose() * Z.inverse(); //18x18 * 18x3 * 3x3               //18x18 * 18x6 * 6x6 = 18x6
    // Correction        
    // internal_state << K * (y-y_hat); //18x3 * (3x1 - 3x1) 
    // internal_state = internal_state + (K * dz); //18x3 * (3x1 - 3x1)            //18x6 * (6x1) 
    internal_state =  (K * dz); //18x3 * (3x1 - 3x1)            //18x6 * (6x1) 

    // New covariance forcing a positive and symetric matrix
    Ptrue = Ptrue - K * Z * K.transpose(); //18x18 - (18x3 * 3x3 * 3x18)        //18x18 - (18x6 * 6x6 * 6x18) = 18x18
    Ptrue = (Ptrue + Ptrue.transpose())*0.5;  //force symmetric
    // std::cout << "Ptrue size: " << Ptrue.size() << std::endl;
    // std::cout << Ptrue << std::endl;
}

void eskf_odometry::correct_state(const Eigen::VectorXf&y, const Eigen::VectorXf&y_hat, const Eigen::MatrixXf& H, const Eigen::MatrixXf& R)
{
    // Innovation   
    Eigen::VectorXf dz  = (y-y_hat);    //1x3                                  //1x6

    //  Observation covariance matrix
    Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(y.size(), y.size());
    Z = H * Ptrue * H.transpose() + R;   //3x18 * 18x18 * 18x3 + 3x3            //6x18 * 18x18 * 18x6 + 6x6 = 6x6
    Z = (Z + Z.transpose())*0.5;

    // Kalman Gain
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(18, y.size());
    K = Ptrue * H.transpose() * Z.inverse(); //18x18 * 18x3 * 3x3               //18x18 * 18x6 * 6x6 = 18x6
    // Correction        
    // internal_state << K * (y-y_hat); //18x3 * (3x1 - 3x1) 
    // internal_state = internal_state + (K * dz); //18x3 * (3x1 - 3x1)            //18x6 * (6x1) 
    internal_state =  (K * dz); //18x3 * (3x1 - 3x1)            //18x6 * (6x1) 

    // New covariance forcing a positive and symetric matrix
    Ptrue = Ptrue - K * Z * K.transpose(); //18x18 - (18x3 * 3x3 * 3x18)        //18x18 - (18x6 * 6x6 * 6x18) = 18x18
    Ptrue = (Ptrue + Ptrue.transpose())*0.5;  //force symmetric
    // std::cout << "Ptrue size: " << Ptrue.size() << std::endl;
    // std::cout << Ptrue << std::endl;
}

Eigen::Quaternionf eskf_odometry::rotVecToQuat(const Eigen::Vector3f& in) {
    float angle = in.norm();
    Eigen::Vector3f axis = (angle == 0) ? Eigen::Vector3f(1, 0, 0) : in.normalized();
    return Eigen::Quaternionf(AngleAxisf(angle, axis));
}

Eigen::Vector4f eskf_odometry::quatToHamilton(const Eigen::Quaternionf& q){
    return (Eigen::Vector4f() <<
            q.coeffs().block<1, 1>(3, 0), // w
            q.coeffs().block<3, 1>(0, 0) // x, y, z
        ).finished();
}

void eskf_odometry::reset_state()
{

    // Update
    std::cout << "internal_state " << internal_state << std::endl;

    external_state.segment(0,6) = external_state.segment(0,6) + internal_state.segment(0,6);    //pos & vel  
    external_state.segment(10,9) = external_state.segment(10,9) + internal_state.segment(9,9);      //ab  wb  g

    Eigen::Quaternionf q_int;
    atools::v2q(internal_state.segment(6,3), q_int);    // internal state to q
    Eigen::Quaternionf q_ext(external_state(6), external_state(7), external_state(8), external_state(9));  // external state to q    
    Eigen::Quaternionf q_prod;
    // global q composition
    atools::qProd(q_int.normalized(), q_ext.normalized(), q_prod);
    // local q composition
    // atools::qProd(q_ext.normalized(), q_int.normalized(), q_prod);
    q_prod = q_prod.normalized();
    // Return vector
    Eigen::Vector4f q_vect(q_prod.w(), q_prod.x(), q_prod.y(), q_prod.z());        
    external_state.segment(6,4) = q_vect;

    // Reset
    internal_state << Eigen::VectorXf::Zero(18);

}



void eskf_odometry::update(Eigen::VectorXf& state, Eigen::VectorXf& covPtrue, Eigen::VectorXf& dxstate)
{
    state = external_state;
    covPtrue = vPtrue;
    dxstate = internal_state;
}





