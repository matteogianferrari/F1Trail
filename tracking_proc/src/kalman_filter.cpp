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

    // Initializes the Camera measurement covariance matrix.
    // The values are set to be the squared variance of the measurement,
    // which is assumed to be +-70mm (depends on the camera used).
    RCamera_ = Eigen::MatrixXd(3, 3);
    RCamera_ << 0.0049, 0., 0.,
                0., 0.0049, 0.,
                0., 0., 0.;

    // Initializes the Lidar measurement covariance matrix.
    // The values are set to be the squared variance of the measurement,
    // which by datasheet is said to be +-40mm (depends on the lidar used).
    RLidar_ = Eigen::MatrixXd(3, 3);
    RLidar_ << 0.0016, 0., 0.,
               0., 0.0016, 0.,
               0., 0., 0.;

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
    noiseAx_ = 2.;
    noiseAy_ = 2.;
}


void KalmanFilter::predict() {
    // State extrapolation equation
    x_ = F_ * x_;   

    // Covariance Extrapolation Equation
    P_ = F_ * P_ * F_.transpose() + Q_;
}


void KalmanFilter::update(const Eigen::VectorXd &z, bool sensorType) {
    // Selects the right measurement covariance matrix based on the sensor
    Eigen::MatrixXd R = (sensorType) ? RCamera_ : RLidar_;

    // Measurement residual (innovation).
    // Difference between the actual measurement `z` and the predicted measurement.
    Eigen::VectorXd y = z - H_ * x_;

    // Innovation covariance matrix.
    // Multiplying by 1e-6 helps with numerical stability during the computation.
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R;
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


Eigen::VectorXd KalmanFilter::getState() {
    Eigen::VectorXd s = Eigen::VectorXd(3);
    s << x_[0], x_[1], x_[2];

    return s;
}


void KalmanFilter::updateMatrices(double dt) {
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    // Updates the update matrix
    F_ << 1., 0., 0., dt, 0., 0.,
          0., 1., 0., 0., dt, 0.,
          0., 0., 1., 0., 0., dt,
          0., 0., 0., 1., 0., 0.,
          0., 0., 0., 0., 1., 0.,
          0., 0., 0., 0., 0., 1.;

    // Updates the process covariance matrix
    Q_ << dt_4 / 4. * noiseAx_, 0., 0., dt_3 / 2. * noiseAx_, 0., 0.,
        0., dt_4 / 4. * noiseAy_, 0., 0., dt_3 / 2. * noiseAy_, 0.,
        0., 0., 0., 0., 0., 0.,
        dt_3 / 2. * noiseAx_, 0., 0., dt_2 * noiseAx_, 0., 0.,
        0., dt_3 / 2. * noiseAy_, 0., 0., dt_2 * noiseAy_, 0.,
        0., 0., 0., 0., 0., 0.;
}
