#ifndef ADVANCED_KALMAN_FILTER_H
#define ADVANCED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <vector>

namespace ultimate_kalman {

    enum class FilterType {
        EKF,
        UKF,
        PF
    };

    class AdvancedKalmanFilter {
    public:
        AdvancedKalmanFilter(FilterType type, int state_dim, int measurement_dim);
        void predict(const Eigen::VectorXd& control_input);
        void update(const Eigen::VectorXd& measurement);
        Eigen::VectorXd getState() const;
        Eigen::MatrixXd getCovariance() const;
        void resetFilter(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance);
        void adaptFilterParameters(const AdaptiveFilteringData& adaptive_data);
        double getInnovationLikelihood() const;
        void performConsistencyCheck();
        void handleMissingMeasurements(const std::vector<bool>& measurement_mask);

    private:
        FilterType type_;
        Eigen::VectorXd x_;  // State estimate
        Eigen::MatrixXd P_;  // Error covariance
        Eigen::MatrixXd F_;  // State transition matrix
        Eigen::MatrixXd H_;  // Measurement matrix
        Eigen::MatrixXd Q_;  // Process noise covariance
        Eigen::MatrixXd R_;  // Measurement noise covariance

        std::vector<Eigen::VectorXd> sigma_points_;
        std::vector<double> particle_weights_;

        void performEKFUpdate(const Eigen::VectorXd& measurement);
        void performUKFUpdate(const Eigen::VectorXd& measurement);
        void performPFUpdate(const Eigen::VectorXd& measurement);
        void generateSigmaPoints();
        void generateParticles();
        void resampleParticles();
    };

} // namespace ultimate_kalman

#endif // ADVANCED_KALMAN_FILTER_H