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
 * @details This class is designed to predict the future state of a system and correct the
 *          predictions based on measurements. It uses a state vector to represent the
 *          object's position and velocity, and matrices to manage uncertainty and model the system's dynamics.
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
     * @param[in]   z Measurement vector (x, y, z).
     */
    void update(const Eigen::VectorXd &z);

    /**
     * @fn          setState
     * 
     * @brief       Set the initial state of the tracked object.
     * 
     * @param[in]   s Tracked object initial position (x, y, z).
     */
    void setState(const Eigen::VectorXd &s);

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
    Eigen::VectorXd x_;     /**< State vector (x, y, z, v_x, v_y, v_z).*/

    Eigen::MatrixXd P_;     /**< Covariance matrix.*/

    Eigen::MatrixXd F_;     /**< Update matrix.*/

    Eigen::MatrixXd Q_;     /**< Process covariance matrix.*/

    Eigen::MatrixXd H_;     /**< Measurement matrix.*/

    Eigen::MatrixXd R_;     /**< Measurement covariance matrix.*/
};

#endif  // KALMAN_FILTER_HPP_
