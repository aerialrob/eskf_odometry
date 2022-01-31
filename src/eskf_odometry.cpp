#include "eskf_odometry.h"


// STD stuff
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>      

// Get the floating point relative accurancy
float EPS = nextafterf(0.0, 1);

using namespace std;

// Additional tools
void q2R(const Eigen::Quaternionf& q, Eigen::Matrix3f& R)
{ R = q.matrix(); }

void v2skew(const Eigen::Vector3f& v, Eigen::Matrix3f& M_sk)
{ M_sk << 0,-v(2,0),v(1,0),v(2,0),0,-v(0,0),-v(1,0),v(0,0),0; }

void v2aaxis(const Eigen::Vector3f& v, float& angle, Eigen::Vector3f& axis)
{
	angle = sqrt(v.dot(v));
	if (angle>EPS)
		axis = v/angle;
	else
	{
		angle = 0.0;
		axis = Eigen::Vector3f::Zero();
	}
}

void aaxis2q(const float& angle, const Eigen::Vector3f& axis, Eigen::Quaternionf& q)
{ q = Eigen::AngleAxisf(angle, axis); }

void v2q(const Eigen::Vector3f& v, Eigen::Quaternionf& q)
{
    float angle;
    Eigen::Vector3f axis;
    v2aaxis(v,angle,axis);
    aaxis2q(angle,axis,q);
}

void qProd(const Eigen::Quaternionf& q1,const Eigen::Quaternionf& q2, Eigen::Quaternionf& q)
{	q = q1*q2; }

void w2omega(const Eigen::Vector3f& w, Eigen::Matrix4f& Omega)
{
	Omega.row(0) << 0.0,-w.transpose();
	Omega.col(0) << 0.0,w;
	Eigen::Matrix3f M_sk(3,3);
	v2skew(-w,M_sk);
	Omega.block(1,1,3,3) = M_sk;
}

void qPredict(const Eigen::Quaternionf& q, const Eigen::Vector3f& w, Eigen::Quaternionf& qpred, const float& dt, const int& met)
{
	Eigen::Quaternionf qn;
	Eigen::Matrix4f Omega;
	Eigen::Vector4f qv;

	switch (met)
	{
		case 0: //Euler method
			w2omega(w,Omega);
			qv << q.w(), q.vec();
			qn = qv + 0.5*dt*(Omega*qv);
			break;
		case 1:	//Exact method
			Eigen::Quaternionf q2;
			v2q(w*dt,q2); 
			qProd(q,q2,qn); // True value - Jacobians based on Euler form
			break;
	}
	qpred = qn.normalized(); // Euler integration - fits with Jacobians
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
    Eigen::Matrix3f M_sk(3,3);    
    Eigen::Vector3f vsk_Ra = R*(a_s-ab);
    v2skew(vsk_Ra, M_sk);
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
    cout << "Low level init " << endl;
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
    cout << "********Print level init*********" << endl;
    cout << "Initial external-State vector (p,v,q,ab,wb,g) size " << external_state.size() << endl;
    cout << "external_state" << endl;
    cout << external_state << endl;
    cout << "# Initial Error-State vector (dp,dv,dtheta,dab,dwb,dg) size " << internal_state.size() << endl;
    cout << "internal_state" << endl;
    cout << internal_state << endl;
    cout << "# System Covariance matrix P size " << internal_state.size() << endl;
    cout << "System Covariance matrix P " << endl;
    cout << Ptrue << endl;

    cout << "# Sensors Parameter Covariance " << internal_state.size() << endl;
    // cout << "Ra_" << endl;
    // cout << Ra_ << endl;
    // cout << "Rw_" << endl;
    // cout << Rw_ << endl;    
    cout << "Rba_" << endl;
    cout << Rba_ << endl;
    cout << "Rbw_" << endl;
    cout << Rbw_ << endl;
    cout << "Rg_" << endl;
    cout << Rg_ << endl;
    cout << "mag_params_vector" << endl;
    cout << mag_params_.magnetic_field_vector[0] << endl;
    cout << mag_params_.magnetic_field_vector[1] << endl;
    cout << mag_params_.magnetic_field_vector[2] << endl;
    cout << endl << "********END Print level init*********" << endl;    
}

int eskf_odometry::set_imu_reading(const float& t_msg, const Eigen::Vector3f& a, const Eigen::Vector3f& w, const Eigen::MatrixXf& Ra, const Eigen::MatrixXf& Rw)
{
    // cout << endl << "IMU reading Low level " << endl;
    // cout << fixed << setprecision(10) << t_msg << " t_msg IMU " << endl;
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
        // cout << fixed << setprecision(10) << t_filter_prev << " t_filter_prev " << endl;
        // cout << fixed << setprecision(10) << t_filter << " t_filter " << endl;
        cout << "FIRST IMU " << t_filter << endl;
        return 1;        
    }    
    if (t_msg < t_filter )
        return 0;

    // Process sensor   
    this->t_filter_prev = t_filter;
    this->t_filter = t_msg;
    // cout << fixed << setprecision(10) << t_filter_prev << " t_filter_prev " << endl;
    // cout << fixed << setprecision(10) << t_filter << " t_filter " << endl;
        
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

    // Covariance Prediction ESKF   error-state       
    cov_predict(external_state, a_, w_, dt_filter, Ptrue);
    vPtrue_ = Ptrue.diagonal();
    Eigen::Vector3f qvector = vPtrue_.segment(6,3);
    Eigen::Quaternionf q;
    v2q(qvector, q); 
    q = q.normalized();   
    Eigen::Vector4f q1_norm(q.w(), q.x(), q.y(), q.z());

    // Cov vector 
    vPtrue << vPtrue_.segment(0,3), vPtrue_.segment(3,3), q1_norm, vPtrue_.segment(9,3), vPtrue_.segment(12,3), vPtrue_.segment(15,3); 

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
    qPredict(quat, w, qpred, dt, 1);   //Exact method qpred normalized        

    // Acceleration in Inertial frame (w/o bias and g). This has to be coherent
    // with Quaternionn integration method. The time step measure must be the same 
    // at k-1 or k
    // Method zero-th backward
    Eigen::Vector3f a;    
    Eigen::Matrix3f R;
    q2R(qpred, R);  
    a = R*(a_s - ab) + g; // 3x3 * 1x3 = 3*1
    // cout << "a " << a << endl;
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
        // cout << "FiQi size" << endl;
        // cout << FiQi.size() << endl;
        // cout << FiQi << endl;


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
    // cout << fixed << setprecision(10) << t_filter_prev << " t_filter_prev " << endl;
    // cout << fixed << setprecision(10) << t_filter << " t_filter " << endl;
    // cout << "--            POS Propagate --" << endl;
    propagate_state();
    // Evaluate mesurement function
    Eigen::VectorXf y_hat = external_state.segment(0,3); // Pos x, y, z from external_state
    // Evaluate mesurement jacobian      
    Eigen::MatrixXf H(3,18);
    H << Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,15);   //(3x18)  
           
    // cout << "--            POS Correct --" << endl;
    correct_state(msg, y_hat, H, R); 
    // cout << "--            POS Reset --" << endl;
    reset_state();

    return 1;
}

//-----------------magnetometer.---------------------
int eskf_odometry::set_magnetometer_reading(const float& t_msg, const Eigen::VectorXf& msg, const Eigen::MatrixXf& R)
{
    if(!is_first_imu_received)   // not first imu call, return
        return -1;
    if (t_msg < t_filter )  // msg time in the past, return
        return 0;

    // Process sensor   
    t_filter_prev = t_filter;
    t_filter = t_msg;
  
    propagate_state();
    // Evaluate mesurement function
    Eigen::Quaternionf qt{external_state[6], external_state[7], external_state[8], external_state[9]};
    Eigen::Matrix3f R_temp;

    Eigen::Vector3f magnetic_field_vector{mag_params_.magnetic_field_vector[0], mag_params_.magnetic_field_vector[1], mag_params_.magnetic_field_vector[2]};   

    // q2R(q,Rot);
    R_temp = qt.matrix();
    // Eigen::Vector3f y_hat = Rot.transpose() * magnetic_field_vector;
        // Magnetic field msg 
    Eigen::Vector3f y_hat;    
    y_hat = R_temp.transpose() * magnetic_field_vector; 
    
    // Evaluate mesurement jacobian      
    Eigen::MatrixXf H(msg.size(),18);       
    Eigen::Matrix3f M_sk(3,3);
    v2skew(magnetic_field_vector, M_sk);

    H << Eigen::MatrixXf::Zero(3,6),  R_temp.transpose() * M_sk, Eigen::MatrixXf::Zero(3,9);
    // H << Eigen::MatrixXf::Zero(3,6),  Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,9);
             
    correct_state(msg, y_hat, H, R); 
    reset_state();

    return 1;
}

//------------------Correction-----------
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
    // cout << "Ptrue size: " << Ptrue.size() << endl;
    // cout << Ptrue << endl;
}

//------------------Reset----------------
void eskf_odometry::reset_state()
{
    // Update
    cout << "internal_state " << internal_state << endl;

    external_state.segment(0,6) = external_state.segment(0,6) + internal_state.segment(0,6);    //pos & vel  
    external_state.segment(10,9) = external_state.segment(10,9) + internal_state.segment(9,9);      //ab  wb  g

    Eigen::Quaternionf q_int;
    v2q(internal_state.segment(6,3), q_int);    // internal state to q
    Eigen::Quaternionf q_ext(external_state(6), external_state(7), external_state(8), external_state(9));  // external state to q    
    Eigen::Quaternionf q_prod;
    // global q composition
    qProd(q_int.normalized(), q_ext.normalized(), q_prod);
    // local q composition
    // qProd(q_ext.normalized(), q_int.normalized(), q_prod);
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





