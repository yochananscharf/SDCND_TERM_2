#include "kalman_filter.h"
#include <math.h>
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;


const double PI  =3.141592653589793238463;


// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
    TODO:
    * predict the state
    */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    //y = z - H * x'
    /**
    TODO:
    * update the state by using Kalman Filter equations
    */
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    //new state

    x_ = x_ + (K * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    //y = z - h(x')
    /**
    TODO:
    * update the state by using Extended Kalman Filter equations
    */
//    float rho = measurement_pack.raw_measurements_[0];
//    float theta = measurement_pack.raw_measurements_[1];
//    float rho = z[0];
//    float theta = z[1];
//    float rho_dot = z[2];
//
//
//    if (theta > PI)
//      theta = fmod((theta - PI) , 2*PI) - PI;
//
//    if (theta < -PI)
//      theta = fmod((theta + PI) , 2*PI) + PI;
//
//    z << rho * cos(theta),rho * sin(theta), rho_dot;
//    MatrixXd Hj = Tools ::CalculateJacobian(x_);
    VectorXd y = z - H_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    //new state

    x_ = x_ + (K * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
