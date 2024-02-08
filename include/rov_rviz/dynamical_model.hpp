//
// Created by morgan on 01/02/24.
// Author: Morgan Louédec
// present the dynamical model of the ROV and define its parameters
//

#ifndef ROV_RVIZ_DYNAMICAL_MODEL_HPP
#define ROV_RVIZ_DYNAMICAL_MODEL_HPP

// the z-position and the attitude of the ROVs are measured,
// but the x and y position must be estimated using a dynamical model

// ---------------------------
// 3 degree of freedom dynamical model of the ROV to predict its horizontal motion
// ---------------------------
// state vector X = [px,py,vu,vv] = [eta,nu]
// in eta, there is
// px : position x
// py : position y
// in nu, there is
// vu : linear velocity along the x-axis in the robot frame
// vv : linear velocity along the y-axis in the robot frame

// dynamical model
// eta_dot = R(yaw)*nu with the transformation matrix R(yaw) and the heading yaw
// nu_dot = M_inv *(tau - D(nu)*nu)
// with the inverse of the rigid mass matrix M_inv,
// the linear+quadratic damping matrix D(nu)
// the control input tau = [fx,fy]

// parameters
double m = 15; // mass of the ROVs (kg)
//I = 0.16; // Inertia of the ROVs (kg.m^2)

Eigen::MatrixXd M = m*Eigen::MatrixXd::Identity(2,2); // rigid mass matrix
//M = Eigen::MatrixXd::Zero(6,6); // rigid mass matrix
//M << m*Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3),
//        Eigen::MatrixXd::Zero(3,3), I*Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd M_inv = M.inverse(); // inverse of the rigid mass matrix

Eigen::MatrixXd DL(2,2); ; // linear damping matrix, diagonal matrix
//DL(0,0) = 4; // x (Ns/m)
//DL(1,1) = 6.2; // y
//DL(2,2) = 5.2; // z
//DL(3,3) = 0.07; // roll (Nms/rad)
//DL(4,4) = 0.07; // pitch
//DL(5,5) = 0.07; // yaw

Eigen::MatrixXd DNL(2,2); // nonlinear damping matrix, diagonal matrix
//DNL(0,0) = 18.2; // x
//DNL(1,1) = 21.7; // y
//DNL(2,2) = 37.7; // z
//DNL(3,3) = 1.55; // roll (Nms/rad)
//DNL(4,4) = 1.55; // pitch
//DNL(5,5) = 1.55; // yaw

// ---------------------------
// Kalman filter to estimate the position of the ROV
// ---------------------------

Eigen::MatrixXd Qp_(4,4); // general process noise covariance matrix
//Qp_(0,0) = 0.1; // px
//Qp_(1,1) = 0.1; // py
//Qp_(2,2) = 0.1; // vu
//Qp_(3,3) = 0.1; // vv

Eigen::MatrixXd Qm_(2,2); // general measurement noise covariance matrix
//Qm_(0,0) = 0.5; // px
//Qm_(1,1) = 0.5; // py

Eigen::MatrixXd P0_ = Eigen::MatrixXd::Identity(4,4); // initial covariance matrix

void init_matrices(){
    DL << 4,0,0,6.2;
    DNL << 18.2,0,0,21.7;
    Qp_ << 0.5,0,0,0,
        0,0.5,0,0,
        0,0,0.5,0,
        0,0,0,0.5;
    Qm_ << 2.,0,0,2.;
}

class ROV_ExtendedKalmanFilter
{
public:
    ROV_ExtendedKalmanFilter(){
        X = Eigen::VectorXd::Zero(4); // default constructor
        P = Eigen::MatrixXd::Identity(4,4); // default constructor
        Qp = Eigen::MatrixXd::Identity(4,4); // default constructor
        Qm = Eigen::MatrixXd::Identity(2,2); // default constructor
    }

    void init(Eigen::VectorXd X0, Eigen::MatrixXd P0, Eigen::MatrixXd Qp, Eigen::MatrixXd Qm, double dt){
        X = X0; // initial estimation
        P = P0; // initial covariance matrix
        this->Qp = Qp; // process noise covariance matrix
        this->Qm = Qm; // measurement noise covariance matrix
        this->dt = dt; // time step
    }

    void predict(const Eigen::VectorXd& tau, double yaw){
        // linearized dynamical model X_dot = F*X + B*tau + Vp with the process noise Vp
        Eigen::VectorXd nu_abs = X.tail(2).cwiseAbs(); // absolute linear velocity vector
        Eigen::MatrixXd Nu_abs(2,2);
        Nu_abs << nu_abs(0),0,
                0,nu_abs(1);

        Eigen::MatrixXd Dnu = -(DL + DNL * Nu_abs);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2,2); // transformation matrix
        R(0,0) = cos(yaw);
        R(0,1) = -sin(yaw);
        R(1,0) = sin(yaw);
        R(1,1) = cos(yaw);

        Eigen::MatrixXd Jf = Eigen::MatrixXd::Zero(4,4); // Jacobian of the continuous dynamical model
        Jf.block(0,2,2,2) = R;
        Jf.block(2,2,2,2) = M_inv * Dnu;

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4,4) + dt*Jf; // Jacobian of the discrete dynamical model

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4,2); // control matrix
        B.block(2,0,2,2) = dt*M_inv;

        X = F*X + B*tau; // state vector derivative
        P = F*P*F.transpose() + Qp; // covariance matrix derivative
    }
    void correct(const Eigen::VectorXd& Y){
        // the measurement is Y(0) = px, Y(1) = py
        // Y = H*X + Vm with the measurement noise Vm
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,4); // Jacobian of the measurement model
        H(0,0) = 1;
        H(1,1) = 1;

        Eigen::MatrixXd S = H*P*H.transpose() + Qm; // innovation covariance matrix
        Eigen::MatrixXd K = P*H.transpose()*S.inverse(); // Kalman gain

        X = X + K*(Y - H*X); // state vector update
        P = (Eigen::MatrixXd::Identity(4,4) - K*H)*P; // covariance matrix update
    }

    Eigen::VectorXd X; // state vector
    Eigen::MatrixXd P; // covariance matrix
    Eigen::MatrixXd Qp; // process noise covariance matrix
    Eigen::MatrixXd Qm; // measurement noise covariance matrix
    double dt; // time step
};
#endif //ROV_RVIZ_DYNAMICAL_MODEL_HPP
