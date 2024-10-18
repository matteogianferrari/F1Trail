#include "kalman_filter.hpp"


KalmanFilter::KalmanFilter() {
    // Initializes the state vector
    x_ = Eigen::VectorXd::Zero(6);

    // Initializes the state covariance matrix.
    // The diagonal values of 1 for the position components (x, y, z) imply that there is
    // high confidence in the initial position estimates (low uncertainty).
    // The diagonal values of 100 for the velocity components (v_x, v_y, v_z) imply that 
    // there is low confidence in the initial velocity estimates (high uncertainty).
    // The off-diagonal elements are set to 0, assuming no correlation between different state variables. 
    P_ = Eigen::MatrixXd(6, 6);
    P_ << 1., 0., 0., 0., 0., 0.,
          0., 1., 0., 0., 0., 0.,
          0., 0., 1., 0., 0., 0.,
          0., 0., 0., 100., 0., 0.,
          0., 0., 0., 0., 100., 0.,
          0., 0., 0., 0., 0., 100.;

    // Initializes the measurement covariance matrix.
    // The diagonal values (0.025) represent the measurement noise variance for the position (x, y, z),
    // indicating the sensor's high accuracy with a low variance.
    // The off-diagonal values are zero, assuming that measurement errors between the axes are independent.
    R_ = Eigen::MatrixXd(3, 3);
    R_ << 0.025, 0., 0.,
          0., 0.025, 0.;
          0., 0., 0.025;

    // Initializes the measurement matrix.
    // Maps the state vector (which includes both position and velocity) to the measurement vector.
    // The measurement vector only contains position (x, y, z), so the corresponding velocity
    // components in the state vector are ignored (set to 0).
    H_ = Eigen::MatrixXd(3, 6);
    H_ << 1., 0., 0., 0., 0., 0.,
          0., 1., 0., 0., 0., 0.,
          0., 0., 1., 0., 0., 0.;

    // Initializes the update matrix.
    // It is set to zeros initially because it depends on the delta time between measurements.
    F_ = Eigen::MatrixXd::Zero(6, 6);

    // Initializes the process covariance matrix.
    // This matrix represents the uncertainty in the system's process model (due to unknown accelerations).
    // It is set to zeros initially because it depends on the delta time between measurements.
    Q_ = Eigen::MatrixXd::Zero(6, 6);

    // Sets the acceleration noise components (no movements assumed on the Z-axis)
    double noise_ax_ = 2.;
    double noise_ay_ = 2.;
}


void KalmanFilter::predict() {
    // State extrapolation equation
    x_ = F_ * x_;   

    // Covariance Extrapolation Equation
    P_ = F_ * P_ * F_.transpose() + Q_;
}


void KalmanFilter::update(const Eigen::VectorXd &z) {
    // Measurement residual (innovation).
    // Difference between the actual measurement `z` and the predicted measurement.
    Eigen::VectorXd y = z - H_ * x_;

    // Innovation covariance matrix.
    // Multiplying by 1e-6 helps with numerical stability during the computation.
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    S += Eigen::MatrixXd::Identity(S.rows(), S.cols()) * 1e-6;

    // Identity matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());

    // Kalman gain equation
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    // State update equation
    x_ = x_ + (K * y);   

    // Covariance update equation
    P_ = (I - K * H_) * P_;
}


void KalmanFilter::setState(const Eigen::VectorXd &s) {
    x_ << s[0], s[1], s[2], 0., 0., 0.;
}


void KalmanFilter::updateMatrices(double dt) {
    double dt_2 = dt_ * dt_;
    double dt_3 = dt_2 * dt_;
    double dt_4 = dt_3 * dt_;

    // Updates the update matrix
    F_ << 1., 0., 0., dt_, 0., 0.,
          0., 1., 0., 0., dt_, 0.,
          0., 0., 1., 0., 0., dt_,
          0., 0., 0., 1., 0., 0.,
          0., 0., 0., 0., 1., 0.,
          0., 0., 0., 0., 0., 1.;

    // Updates the process covariance matrix
    Q_ << dt_4 / 4. * noise_ax_, 0., 0., dt_3 / 2. * noise_ax_, 0., 0.,
        0., dt_4 / 4. * noise_ay_, 0., 0., dt_3 / 2. * noise_ay_, 0.,
        0., 0., 0., 0., 0., 0.,
        dt_3 / 2. * noise_ax_, 0., 0., dt_2 * noise_ax_, 0., 0.,
        0., dt_3 / 2. * noise_ay_, 0., 0., dt_2 * noise_ay_, 0.,
        0., 0., 0., 0., 0., 0.;
}
