#include "eskf_odometry.h"
#include "rot_fc.h"


// STD stuff
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>      


//-----------------Help Functions---------------------

/**
* \brief Quaternion to rotation matrix
*
*  Returns the rotation matrix from a quaternion 
* Input:
*   q:    Quaternion with convenion: q = [qw qx qy qz] with qw th scalar element.
* Output:
*   R:    Rotation matrix.
*/
void q2R(const Eigen::Quaternionf& q,  Eigen::Matrix3f& R)
{
    R = q.matrix();
}

/**
* \brief Rotation vector to quaternion
*
* Rotation vector to quaternion 
*
* Input:
*   v:    Rotation vector.
* Output:
*   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element. 
*/
void vec2q (const Eigen::Vector3f& v, Eigen::Quaternionf& q)
{    
    // Get angle and axis 
    float angle = sqrt(v.dot(v));
    Eigen::Vector3f axis;
    float EPS_ = std::nextafterf(0.0, 1);

    if (angle < EPS_){
        angle = 0.0;
        std::cout << "Angle too small " << angle << std::endl;  
        axis << 1,0,0;
    }
    else
        axis =  v/angle;
    // Create q from angle and axis        
    q = Eigen::Quaternionf{Eigen::AngleAxisf{angle, axis}};        
}

/**
* \brief Quaternion multiplication
*
* Rotation vector to quaternion 
*
* Input:
*   q1:   Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
*   q2:   Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
* Output:
*   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
* 
*/
Eigen::Quaternionf quatMult(Eigen::Quaternionf q1, Eigen::Quaternionf q2) {
    Eigen::Quaternionf resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}

/**
* \brief  Euler angles to quaternion conversion
*
* Returns the quaternion representing the specified Euler angles 
*
* Input:
*   e:    Euler angles: e = [roll pitch yaw]'
* Output:
*   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
*/
void e2q(const Vector3f& e, Quaternionf& q)
{
	q = Quaternionf(AngleAxisf(e(2), Vector3f::UnitZ())*
				          AngleAxisf(e(1), Vector3f::UnitY())*
				          AngleAxisf(e(0), Vector3f::UnitX()));

}

/**
* \brief Vector to skew symmetric matrix
*
* Vector to skew symmetric matrix
*
* Input:
*	vec: 3x1 vector
*
* Output:
*	M_sk: Skew symmetric matrix (1rst row: 0 -z -y)
*/
Eigen::Matrix3f v2skew(const Eigen::Vector3f& vec)  
{
    Eigen::Matrix3f M_sk;
    // M_sk << 0, -vec(3), vec(2),
    //         vec(3), 0, -vec(1),
    //         -vec(2), vec(1), 0;

    M_sk << 0,-vec(2,0),vec(1,0),
            vec(2,0),0,-vec(0,0),
            -vec(1,0),vec(0,0),0;            

    return  M_sk;
}

/**
* \brief Quaternion to Euler angles conversion
*
* Returns the Euler angles representing the specified quaternion and 
* its Jacobian wrt q if requested.
*
* Input:
*   q:    Quaternion with convenion: q = [qw qx qy qz] with qw the scalar element.
* Output:
*   e:    Euler angles: e = [roll pitch yaw]'.
*   E_q (optional):  Jacobian wrt quaternion.
*/
void q2e(const Eigen::Quaternionf& q, Eigen::Vector3f& e, Eigen::MatrixXf& E_q)
{
	float a,b,c,d;
	a=q.w();
	b=q.x();
	c=q.y();
	d=q.z();

	float y1,x1,z2,y3,x3;
	y1 =  2.0*c*d + 2.0*a*b;
	x1 =  a*a - b*b - c*c + d*d;
	z2 = -2.0*b*d + 2.0*a*c;
	y3 =  2.0*b*c + 2.0*a*d;
	x3 =  a*a + b*b - c*c - d*d;

    e << atan2(y1,x1),asin(z2),atan2(y3,x3);	

    Eigen::Vector4f dx1dq,dy1dq,dz2dq,dx3dq,dy3dq,de1dq,de2dq,de3dq;
    dx1dq << 2.0*a,-2.0*b,-2.0*c, 2.0*d;
    dy1dq << 2.0*b, 2.0*a, 2.0*d, 2.0*c;
    dz2dq << 2.0*c,-2.0*d, 2.0*a,-2.0*b;
    dx3dq << 2.0*a, 2.0*b,-2.0*c,-2.0*d;
    dy3dq << 2.0*d, 2.0*c, 2.0*b, 2.0*a;

    float de1dx1,de1dy1,de2dz2,de3dx3,de3dy3;
    de1dx1 = -y1/(x1*x1 + y1*y1);
    de1dy1 =  x1/(x1*x1 + y1*y1);
    de2dz2 = 1.0/sqrt(1.0-z2*z2);
    de3dx3 = -y3/(x3*x3 + y3*y3);
    de3dy3 =  x3/(x3*x3 + y3*y3);

    E_q = Eigen::MatrixXf::Zero(3,4);
    E_q.row(0) = de1dx1*dx1dq + de1dy1*dy1dq;
    E_q.row(1) = de2dz2*dz2dq;
    E_q.row(2) = de3dx3*dx3dq + de3dy3*dy3dq;
}


// IMU transition matrix from error state (ESKF) 
Eigen::MatrixXf imu_trans_mat(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt)
{    
    // Derivative r.t. Error-State _____________________________________________
    Eigen::VectorXf q_ = xstate.segment(6,4);
    Eigen::VectorXf ab = xstate.segment(10,3);
    Eigen::VectorXf wb = xstate.segment(13,3);
    Eigen::Quaternionf q{q_(0), q_(1), q_(2), q_(3)};
    Eigen::Matrix3f R;
    q2R(q, R);         

    // Product on the left hand side of the nominal quaternion
    // Global Error (GE)
    Eigen::Matrix3f V = -v2skew(R*(a_s-ab)); //3x3 * 1x3    
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

void eskf_odometry::set_init_params(const params& f_params, const Eigen::VectorXf& xtrue0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu, const Sensor::position_params& position, const Sensor::orientation_params& orientation)
{
    std::cout << "Low level init " << std::endl;
    // Initial State
    f_params_ = f_params;
    external_state = xtrue0;
    internal_state = dx0;
    std::cout << "external_state" << std::endl;
    std::cout << external_state << std::endl;

    std::cout << "internal_state" << std::endl;
    std::cout << internal_state << std::endl;
    imu_ = imu;
    position_ = position;
    orientation_ = orientation;    

    // Initial Covariance
    Eigen::VectorXf std_ini = Eigen::VectorXf(18);   

    std_ini << f_params_.dp0_std, f_params_.dv0_std, f_params_.dtheta0_std, f_params_.dab0_std, f_params_.dwb0_std, f_params_.dg0_std;      
    std::cout << "std_ini" << std::endl;    
    std_ini = std_ini.array() * std_ini.array();

    Eigen::MatrixXf Ptrue0;
    Ptrue0 = std_ini.asDiagonal();
    std::cout << "Ptrue0" << std::endl;
    std::cout << Ptrue0 << std::endl;

    Ptrue = Ptrue0;

}

void eskf_odometry::print_ini_params()
{
    std::cout << "Print level init " << std::endl;
    std::cout << "Initial Nominal-State vector (p,v,q,ab,wb,g) size " << external_state.size() << std::endl;
    std::cout << "# Initial Error-State vector (dp,dv,dtheta,dab,dwb,dg) size " << dx0_.size() << std::endl;
}




// %   Inputs:
// %       - state:        State vector.
// %       - frame:        Frame where the orientation error is expressed.
// %       - params:       Sensor parameters.
// %       - std_pose:     Noise std dev.
// %
// %   Ouputs:
// %       - h:         Sensor readings
// %       - H_xstate:  Observation Jacobian w.r.t. nominal state (EKF)
// %       - H_dxstate: Observation Jacobian w.r.t. error state (ESKF)

void pose_obs_model(const Eigen::VectorXf& xstate, const Eigen::VectorXf& std_pose, Eigen::VectorXf& h, Eigen::MatrixXf& H_dxstate, Eigen::MatrixXf& H_xstate)
{
    //std_pose //std_pose=zeros(6,1);
    Eigen::VectorXf p = xstate.segment(0,3);
    Eigen::Quaternionf q{xstate[6], xstate[7], xstate[8], xstate[9]};
    Eigen::Vector3f euler_angles;
    Eigen::MatrixXf J_q;
    q2e(q, euler_angles, J_q);

    Eigen::VectorXf prev_h(h.size());    
        
    prev_h << p, euler_angles;

    // Observation model      
    h = prev_h + (std::rand() % 10 + 1)*std_pose;

    Eigen::MatrixXf M0 = Eigen::MatrixXf::Zero(3,3);
    Eigen::MatrixXf MIdent = Eigen::MatrixXf::Identity(3,3);

    // Observation Jacobian 
    H_dxstate << Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,15),
                                 Eigen::MatrixXf::Zero(3,6), Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,9);
    H_xstate << Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,16),
                                 Eigen::MatrixXf::Zero(3,6), J_q, Eigen::MatrixXf::Zero(3,9);                                 
    
}

void eskf_odometry::innovation(const Eigen::VectorXf& y, const Eigen::VectorXf& e, const Eigen::MatrixXf& P_old, const Eigen::MatrixXf& H, const Eigen::MatrixXf& Q,
                Eigen::VectorXf& dz, Eigen::MatrixXf& Z)
{



    
}


//bool
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

    std::cout << "-- IMU Propagate --" << std::endl;
    propagate_state();    
    return 1;
}

//----------------- Mean State Prediction and Covariance Prediction ESKF   error-state ---------------------
void eskf_odometry::propagate_state()  
{
    // dt
    float dt_filter = t_filter - t_filter_prev;

    // Prediction __________________________________________________           
    // Mean State Prediction
    external_state = mean_predict(external_state, a_, w_, dt_filter);
    // std::cout << xstate_nom << std::endl;
    // std::cout << "Nominal-State vector (p,v,q,ab,wb,g) size " << external_state.size() << std::endl;
    // std::cout << "Nominal-State vector (p,v,q,ab,wb,g) size " << xstate_nom.size() << std::endl;

    // Covariance Prediction ESKF   error-state
    std::cout << "a_ " << std::endl;
    std::cout << a_ << std::endl;
    std::cout << "w_ " << std::endl;
    std::cout << w_ << std::endl;

    cov_predict(external_state, a_, w_, dt_filter, Ptrue);


    // Ptrue = cov_predict(Ptrue, external_state, a_, w_, dt_filter);
    // std::cout << "Prediction ESKF size " << Ptrue.size() << std::endl;
    // std::cout <<  Ptrue << std::endl;
    // To compose matrix
    vPtrue_ = Ptrue.diagonal();
    Eigen::VectorXf qvector = vPtrue_.segment(6,3);
    Eigen::Quaternionf q;
    vec2q((qvector).normalized(), q);    
    Eigen::Vector4f q1_norm(q.w(), q.x(), q.y(), q.z());

    // Eigen::VectorXf p = vPtrue_.segment(0,3);
    // Eigen::VectorXf v = vPtrue_.segment(3,3);
    // Eigen::VectorXf q = q1_norm;
    // Eigen::VectorXf ab = vPtrue_.segment(9,3);
    // Eigen::VectorXf wb = vPtrue_.segment(12,3);
    // Eigen::VectorXf g = vPtrue_.segment(15,3);

    vPtrue << vPtrue_.segment(0,3), vPtrue_.segment(3,3), q1_norm, vPtrue_.segment(9,3), vPtrue_.segment(12,3), vPtrue_.segment(15,3);
    // std::cout << "vPtrue  " << vPtrue << std::endl;

}

//----------------- Mean State Prediction ---------------------
Eigen::VectorXf eskf_odometry::mean_predict(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt)
{
    // std::cout << std::endl << "Propagate Low level " << std::endl;

    // Propagate Bias and gravity (considering linear during integration) ___
    Eigen::VectorXf p = xstate.segment(0,3);
    Eigen::VectorXf v = xstate.segment(3,3);
    Eigen::VectorXf q = xstate.segment(6,4);
    Eigen::VectorXf ab = xstate.segment(10,3);
    Eigen::VectorXf wb = xstate.segment(13,3);
    Eigen::VectorXf g = xstate.segment(16,3);

    // Angular velocity in Inertial frame (= body Frame, w/o bias)
    Eigen::Vector3f w = w_s - wb;       //eq (22c)

    // % Relative rotation due to last angular velocity
    Eigen::VectorXf q_nom = qPredict(q,w,dt);
    // std::cout << "Q_predict " << std::endl;
    // std::cout << q_nom << std::endl;

    // Acceleration in Inertial frame (w/o bias and g). This has to be coherent
    // with Quaternionn integration method. The time step measure must be the same 
    // at k-1 or k
    // Method zero-th backward
    Eigen::Vector3f a;
    Eigen::Quaternionf q_nom_eigen{q_nom(0), q_nom(1), q_nom(2), q_nom(3)};
    Eigen::Matrix3f R;
    q2R(q_nom_eigen,R);  
    a = R*(a_s - ab)+g; // 3x3 * 1x3 = 3*1
    // std::cout << "a " << a << std::endl;
    // Position and Velocity
    Eigen::VectorXf p_nom;
    Eigen::VectorXf v_nom;    

    int trunc_met = 1;  // Truncated method 
    p_nom = p + v*dt; // Linear position in Inertial frame (integrating vel. and acc.)
    v_nom = v + a*dt; // Linear velocity in Inertial frame (integrating acc.)
    // Return vector
    // xstate_nom = [p_nom;v_nom;q_nom;ab;wb;g]; % Nominal state
    Eigen::VectorXf xstate_nom(xstate.size());
    xstate_nom << p_nom, v_nom, q_nom, ab, wb, g;

    // Componer msg ROS
    return xstate_nom;    
}

//----------------- q prediction step for the nominal-state vector.------------------
Eigen::VectorXf eskf_odometry::qPredict(const Eigen::VectorXf& q_, const Eigen::Vector3f& w, const float& dt)
{
    //Method
    //zero-th for-ward, zero-th backward and first order integrators
    char met = 'b'; 
    // Eigen::VectorXf q1;
    Eigen::Quaternionf q{q_(0), q_(1), q_(2), q_(3)};
    // zero-th backward
    // std::cout << "Q integration Method = Q0B" << std::endl;  
    Eigen::Quaternionf q1;
    Eigen::Quaternionf q_w_dt;
    vec2q((w*dt), q_w_dt);
    q1 = quatMult(q, q_w_dt).normalized();   
    // Return vector
    Eigen::Vector4f q1_norm(q1.w(), q1.x(), q1.y(), q1.z());
    return q1_norm;
}

//-----------------Imu Covariance Error Propagation  ESKF Transition matrix.---------------------
// Eigen::MatrixXf eskf_odometry::cov_predict(const Eigen::VectorXf& P_old, const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt)
void eskf_odometry::cov_predict(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt, Eigen::MatrixXf& P_old)
{
    // Covariance Error Propagation
    // IMU transition matrix from error state (ESKF) 
    Eigen::MatrixXf F_dxstate(18,18);
    //- F_dxstate:           ESKF Transition matrix.
    F_dxstate = imu_trans_mat(xstate, a_s, w_s, dt);
    // std::cout << "F_dxstate" << std::endl;
    // std::cout << F_dxstate << std::endl;

    //- P_old:        Covariance matrix at time k-1.    
    //- Fi:           Jacobian that maps the IMU covariance.
    //- Qi:           Imu covariance matrix.
    //- P:            Covariance matrix at time k.
    Eigen::MatrixXf P(18,18);     
    Qi << Ra_ , Eigen::MatrixXf::Zero(3,9), 
        Eigen::MatrixXf::Zero(3,3), Rw_, Eigen::MatrixXf::Zero(3,6),
        Eigen::MatrixXf::Zero(6,12); 

    std::cout << "Qi" << std::endl;
    std::cout << Qi << std::endl;
    Fi << Eigen::MatrixXf::Zero(3,12), Eigen::MatrixXf::Identity(12,12), Eigen::MatrixXf::Zero(3,12);
        
    FiQi = Fi*Qi*Fi.adjoint();
    std::cout << "FiQi" << std::endl;
    std::cout << FiQi << std::endl;

    P_old = F_dxstate * P_old * F_dxstate.adjoint() + FiQi; // 18*18    transpose()
    P_old = (P_old + P_old.adjoint()) * 0.5;    //force symmetric
    std::cout << "P_old" << std::endl;
    std::cout << P_old << std::endl;
    // return P;



    // P = F_dxstate * P_old * F_dxstate.adjoint() + FiQi; // 18*18    transpose()
    // P = (P + P.adjoint()) * 0.5;    //force symmetric
    // std::cout << "P" << std::endl;
    // std::cout << P << std::endl;
    // return P;
}

int eskf_odometry::set_position_reading(const float& t_msg, const Eigen::VectorXf& msg, const Eigen::MatrixXf& R)
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
    std::cout << "-- POS Propagate --" << std::endl;
    propagate_state();
    // Evaluate mesurement function
    Eigen::VectorXf y_hat = external_state.segment(0,3); // Pos x, y, z from external_state
    // Evaluate mesurement jacobian      
    Eigen::MatrixXf H(msg.size(),18);
    H << Eigen::MatrixXf::Identity(msg.size(),msg.size()), Eigen::MatrixXf::Zero(msg.size(),15);   //(3x18)  
           
    std::cout << "-- POS Correct --" << std::endl;
    correct_state(msg, y_hat, H, R); 
    std::cout << "-- POS Reset --" << std::endl;
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
    // std::cout << std::fixed << std::setprecision(10) << t_filter_prev << " t_filter_prev " << std::endl;
    // std::cout << std::fixed << std::setprecision(10) << t_filter << " t_filter " << std::endl;
    std::cout << "-- POSE Propagate --" << std::endl;
    propagate_state();
    // Evaluate mesurement function
    Eigen::Quaternionf q{external_state[6], external_state[7], external_state[8], external_state[9]};
    Eigen::Matrix3f Rot;
    Eigen::Vector3f magnetic_field_vector{1,1,1};   
    q2R(q,Rot);
    Eigen::Vector3f y_hat = Rot.transpose() * magnetic_field_vector;

    
    // Evaluate mesurement jacobian      
    Eigen::MatrixXf H(msg.size(),18);
    H << Eigen::MatrixXf::Zero(3,6),  Rot.transpose() * v2skew(magnetic_field_vector), Eigen::MatrixXf::Zero(3,9);

         
    std::cout << "-- POSE Correct --" << std::endl;
    correct_state(msg, y_hat, H, R); 
    std::cout << "-- POSE Reset --" << std::endl;
    reset_state();

    return 1;
}


void eskf_odometry::correct_state(const Eigen::VectorXf&y, const Eigen::VectorXf&y_hat, const Eigen::MatrixXf& H, const Eigen::MatrixXf& R)
{
    // std::cout << "Ptrue " << Ptrue.size() << std::endl;
    // std::cout << Ptrue << std::endl;
    // std::cout << "Z " << Z.size() << std::endl;
    // std::cout << Z << std::endl;
    // std::cout << "y size " << y.size() << std::endl;
    // std::cout << "x " << y(0) << std::endl;
    // std::cout << "y " << y(1) << std::endl;
    // std::cout << "z " << y(2) << std::endl;
    // Kalman Gain
    Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(y.size(), y.size());
    Z << H * Ptrue * H.transpose() + R;   //3x18 * 18x18 * 18x3 + 3x3
    Z = (Z + Z.transpose())*0.5;
    std::cout << "Z size: " << Z.size() << std::endl;
    std::cout << Z << std::endl;

    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(18, y.size());
    K << Ptrue * H.transpose() * Z.inverse(); //18x18 * 18x3 * 3x3
    std::cout << "K size: " << K.size() << std::endl;
    std::cout << K << std::endl;
    // Calculate posterior state vector and covariance matrix
    // Correction        
    // std::cout << "y_haat size " << y_hat.size() << std::endl;
    // std::cout << "x " << y_hat(0) << std::endl;
    // std::cout << "y " << y_hat(1) << std::endl;
    // std::cout << "z " << y_hat(2) << std::endl;
    internal_state << K * (y-y_hat); //18x3 * (3x1 - 3x1) 

    // New covariance forcing a positive and symetric matrix
    Ptrue = Ptrue - K * Z * K.transpose(); //18x18 - (18x3 * 3x3 * 3x18)
    Ptrue = (Ptrue + Ptrue.transpose())*0.5;  //force symmetric
    // std::cout << "Ptrue size: " << Ptrue.size() << std::endl;
    // std::cout << Ptrue << std::endl;
}

void eskf_odometry::reset_state()
{
    std::cout << "internal_state " << internal_state << std::endl;
    

    external_state.segment(0,3) = external_state.segment(0,3) + internal_state.segment(0,3);    //pos
    external_state.segment(3,3) = external_state.segment(3,3) + internal_state.segment(3,3);    //vel
    Eigen::Quaternionf q_int;
    vec2q(internal_state.segment(6,3), q_int);    // internal state to q
    Eigen::Quaternionf q_ext{external_state(6), external_state(7), external_state(8), external_state(9)};  // external state to q    
    Eigen::Quaternionf q_mult;
    q_mult = quatMult (q_int.normalized(), q_ext.normalized()).normalized();   
    // Return vector
    Eigen::Vector4f q_vect(q_mult.w(), q_mult.x(), q_mult.y(), q_mult.z());        
    external_state.segment(6,4) = q_vect;      // GE q composition
    external_state.segment(10,3) = external_state.segment(10,3) + internal_state.segment(9,3);      //ab
    external_state.segment(13,3) = external_state.segment(13,3) + internal_state.segment(12,3);     //wb
    external_state.segment(16,3) = external_state.segment(16,3) + internal_state.segment(15,3);     //g

}



void eskf_odometry::update(Eigen::VectorXf& state, Eigen::VectorXf& covPtrue)
{
    state = external_state;
    covPtrue = vPtrue;
}





