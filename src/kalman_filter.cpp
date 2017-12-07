#include "kalman_filter.h"
#include <math.h>
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;


const float PI  =3.14159;


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
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    float eps = 0.000001;  // Make sure we don't divide by 0.
    if (px < eps && py < eps) {
        px = eps;
        py = eps;
    } else if (px < eps) {
        px = eps;
    }


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

    float eps = 0.000001;  // Make sure we don't divide by 0.

    float ro = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    float phi = eps;
    if (fabs(x_[0]) > eps) {
        phi = atan2(x_[1], x_[0]);
    }
    float ro_dot = eps;
    if (fabs(ro) > eps) {
        ro_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / ro;
    }

    VectorXd hx(3);
    hx << ro, phi, ro_dot;

    VectorXd y = z - hx;

    y[1] = atan2(sin(y[1]), cos(y[1]));

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
