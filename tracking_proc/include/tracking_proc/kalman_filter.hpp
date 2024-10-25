/**
 * @file    kalman_filter.hpp
 * 
 * @author  Matteo Gianferrari
 *
 * @date    2024-10-17
 * 
 * @brief   Header file for the Kalman Filter implementation.
 */
#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <eigen3/Eigen/Dense>

/**
 * @class   KalmanFilter
 * 
 * @brief   Implements a Kalman Filter for estimating and tracking the state of an object.
 * 
 * @details This class implements a Kalman Filter using the constant velocity model to predict and correct
 *          the state of an object over time. The Kalman Filter estimates the position and velocity of the
 *          object and updates its predictions using new measurements. The class uses a state vector to 
 *          represent the object's position and velocity, while leveraging matrices to manage uncertainty
 *          and model the system's dynamics effectively.
 */
class KalmanFilter
{
public:
    /**
     * @fn          KalmanFilter 
     * 
     * @brief       Construct a new Kalman Filter object.
     */
    KalmanFilter();

    /**
     * @fn      predict
     * 
     * @brief   Estimates the future state of the tracked object.
     */
    void predict();

    /**
     * @fn          update
     * 
     * @brief       Corrects the estimation with the actual measurement.
     * 
     * @details     The update function uses a bool to identify the sensor type.
     *              The update process requires the use of the measurement covariance matrix,
     *              which is defined for each sensor (camera and lidar). The bool is used
     *              to assign the correct measurement covariance matrix to a local matrix R,
     *              making the Innovation covariance matrix S computation easier.
     *
     * @param[in]   z Measurement vector (x, y, z).
     * @param[in]   sensorType True for Camera measurement, false for Lidar measurement.
     */
    void update(const Eigen::VectorXd &z, bool sensorType);

    /**
     * @fn          setState
     * 
     * @brief       Set the initial state of the tracked object.
     * 
     * @details     Only the position of the tracked object is known, thus
     *              the state vector is initialized with velocity equals to 0.
     *
     * @param[in]   s Tracked object initial position (x, y, z).
     */
    void setState(const Eigen::VectorXd &s);

    /**
     * @brief   Gets the state of the tracked object.
     * 
     * @details Only the position of the tracked object is required.
     *
     * @return  Eigen::VectorXd Position of the tracked object (x, y, z).
     */
    Eigen::VectorXd getState();

    /**
     * @fn          updateMatrices
     *
     * @brief       Updates the matrices that are dependent on a delta time.
     *
     * @details     Updates the update matrix F and the process covariance matrix Q.
     *              This step is necessary to account for changes in the object's dynamics over time.
     * 
     * @param[in]   dt Delta time between measurements.
     */
    void updateMatrices(double dt);

private:
    Eigen::VectorXd x_; /**< State vector (x, y, z, v_x, v_y, v_z).*/

    Eigen::MatrixXd P_; /**< Covariance matrix.*/

    Eigen::MatrixXd F_; /**< Update matrix.*/

    Eigen::MatrixXd Q_; /**< Process covariance matrix.*/

    /**
     * @brief Measurement matrix for camera and lidar (both measure the same components).
     */
    Eigen::MatrixXd H_; 

    Eigen::MatrixXd RCamera_;   /**< Measurement covariance matrix for the camera.*/
    Eigen::MatrixXd RLidar_;    /**< Measurement covariance matrix for the lidar.*/

    double noiseAx_;    /**< Acceleration noise component for the X axis.*/
    double noiseAy_;    /**< Acceleration noise component for the Y axis.*/
};

#endif  // KALMAN_FILTER_HPP_
