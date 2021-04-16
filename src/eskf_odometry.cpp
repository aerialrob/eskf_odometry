#include "eskf_odometry.h"

// STD stuff
#include <fstream>
#include <iostream>
#include <sstream>

#include <math.h>      

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
// Rotation vector to quaternion conversion.
Eigen::Quaternionf vec2q (const Eigen::Vector3f& w)
{    
    // Get angle and axis 
    float angle = sqrt(w.dot(w));
    Eigen::Vector3f axis;
    float EPS_ = std::nextafterf(0.0, 1);

    if (angle < EPS_){
        angle = 0.0;
        std::cout << "Angle too small " << angle << std::endl;  
        axis << 1,0,0;
    }
    else
        axis =  w/angle;
    // Create q from angle and axis        
    Eigen::Quaternionf q_eigen = Eigen::Quaternionf{Eigen::AngleAxisf{angle, axis}};    
    return q_eigen;
}
// Quaternion multiplication
Eigen::Quaternionf quatMult(Eigen::Quaternionf q1, Eigen::Quaternionf q2) {
    Eigen::Quaternionf resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}



// Class methods
eskf_odometry::eskf_odometry()
{

}

void eskf_odometry::set_init_params(const params& f_params, const Eigen::VectorXf& x0, const Eigen::VectorXf& dx0, const Sensor::imu_params& imu,const Sensor::pose_params& pose,const Sensor::pose_params& pose2,const Sensor::position_params& position,const Sensor::orientation_params& orientation,const Sensor::linvel_params& linvel)
{
    std::cout << "Low level init " << std::endl;
    f_params_ = f_params;
    xtrue = x0;
    dx0_ = dx0;
    imu_ = imu;
    pose_ = pose; 
    pose2_ = pose2;
    position_ = position;
    orientation_ = orientation;
    linvel_ = linvel;
    // TO DO Init Covariance matrice diag(std_ini.^2);
    // Ptrue ;
    // TO DO fill the initial covariance matrice with std values
    // Eigen::MatrixXf Ptrue = Eigen::MatrixXf::Zero(18,18);
 
    Eigen::VectorXf std_est_imu_cont(12);
    std_est_imu_cont = imu_.std * imu_.std;
    Qi = std_est_imu_cont.asDiagonal();      
    std::cout << Qi << std::endl;

    Fi << Eigen::MatrixXf::Zero(3,12), Eigen::MatrixXf::Identity(12,12), Eigen::MatrixXf::Zero(3,12);
    std::cout << Fi << std::endl;
    
    FiQi = Fi*Qi*Fi.adjoint();
    std::cout << FiQi << std::endl;
    // [a_est_std_cont;w_est_std_cont;ab_est_std;wb_est_std]; %a,w,ab,wb
    // Eigen::MatrixXf Fi = Eigen::MatrixXf::Zero(18,12);
    // Fi(4:15,:)=eye(12);

}

void eskf_odometry::print_ini_params()
{
    std::cout << "Print level init " << std::endl;
    std::cout << "Initial Nominal-State vector (p,v,q,ab,wb,g) size " << xtrue.size() << std::endl;
    std::cout << "# Initial Error-State vector (dp,dv,dtheta,dab,dwb,dg) size " << dx0_.size() << std::endl;
}

//Skew-symmetric matrix
Eigen::Matrix3f vec2skew(const Eigen::Vector3f& vec)  
{
    Eigen::Matrix3f M_sk;
    M_sk << 0, -vec(3), vec(2),
            vec(3), 0, -vec(1),
            -vec(2), vec(1), 0;

    return  M_sk;
}

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
    Eigen::Matrix3f V = -vec2skew(R*(a_s-ab)); //3x3 * 1x3    
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

Eigen::MatrixXf eskf_odometry::cov_predict(const Eigen::VectorXf& P_old, const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt)
{
    // Covariance Error Propagation
    // IMU transition matrix from error state (ESKF) 
    Eigen::MatrixXf F_dxstate(18,18);
    F_dxstate = imu_trans_mat(xstate, a_s, w_s, dt);
    std::cout << F_dxstate << std::endl;

    //- P_old:        Covariance matrix at time k-1.
    //- F_dxstate:           ESKF Transition matrix.
    //- Fi:           Jacobian that maps the IMU covariance.
    //- Qi:           Imu covariance matrix.
    //- P:            Covariance matrix at time k.
    Eigen::MatrixXf P(18,18);
    P = F_dxstate*P_old*F_dxstate.adjoint() + FiQi; // 18*18
    P = (P + P.adjoint())*0.5;    //force symmetric
    return P;
}


void eskf_odometry::set_imu_reading(const float& dt_imu, const Eigen::Vector3f& a, const Eigen::Vector3f& w)
{
    std::cout << "IMU reading Low level " << std::endl;
    // Prediction __________________________________________________
    // Mean State Prediction
    // Eigen::VectorXf xstate_nom;
    xtrue = mean_predict(xtrue,a,w,dt_imu);

    // std::cout << xstate_nom << std::endl;
    std::cout << "Initial Nominal-State vector (p,v,q,ab,wb,g) size " << xtrue.size() << std::endl;
    // std::cout << "Nominal-State vector (p,v,q,ab,wb,g) size " << xstate_nom.size() << std::endl;

    // Covariance Prediction ESKF
    Ptrue = cov_predict(Ptrue,xtrue,a,w,dt_imu);
    std::cout << "Prediction ESKF size " << Ptrue.size() << std::endl;
    // std::cout <<  Ptrue << std::endl;
    vPtrue_ = Ptrue.diagonal();
    Eigen::VectorXf qvector = vPtrue_.segment(6,3);
    Eigen::Quaternionf q = vec2q(qvector).normalized();    
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

Eigen::VectorXf eskf_odometry::mean_predict(const Eigen::VectorXf& xstate, const Eigen::Vector3f& a_s, const Eigen::Vector3f& w_s, const float& dt)
{
    std::cout << "On mean predict Low level " << std::endl;

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
    std::cout << "Q_predict " << std::endl;
    std::cout << q_nom << std::endl;

    // Acceleration in Inertial frame (w/o bias and g). This has to be coherent
    // with Quaternionn integration method. The time step measure must be the same 
    // at k-1 or k

    // Method zero-th backward
    Eigen::Vector3f a;
    Eigen::Quaternionf q_nom_eigen{q_nom(0), q_nom(1), q_nom(2), q_nom(3)};
    Eigen::Matrix3f R;
    q2R(q_nom_eigen,R);  
    a = R*(a_s - ab)+g; // 3x3 * 1x3 = 3*1
    std::cout << "a " << a << std::endl;
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
    return xstate_nom;
}

Eigen::VectorXf eskf_odometry::qPredict(const Eigen::VectorXf& q_, const Eigen::Vector3f& w, const float& dt)
{
    //Method
    //zero-th for-ward, zero-th backward and first order integrators
    char met = 'b'; 
    // Eigen::VectorXf q1;
    Eigen::Quaternionf q{q_(0), q_(1), q_(2), q_(3)};
    // zero-th backward
    std::cout << "Q integration Method = Q0B" << std::endl;  
    Eigen::Quaternionf q1;
    q1 = quatMult(q, vec2q(w*dt)).normalized();   
    // Return vector
    Eigen::Vector4f q1_norm(q1.w(), q1.x(), q1.y(), q1.z());
    return q1_norm;
}

void eskf_odometry::update(Eigen::VectorXf& state, Eigen::VectorXf& covPtrue)
{
    state = xtrue;
    covPtrue = vPtrue;
}





