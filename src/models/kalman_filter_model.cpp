#include "advanced_crewai_kalman_filter_system.h"
#include <stdexcept>
#include <cmath>

namespace advanced_kalman {

class StandardKalmanFilterModel : public KalmanFilterModel {
public:
    StandardKalmanFilterModel(int state_dim, int measurement_dim)
        : state_dim_(state_dim), measurement_dim_(measurement_dim) {
        x_ = VectorXd::Zero(state_dim);
        P_ = MatrixXd::Identity(state_dim, state_dim);
        F_ = MatrixXd::Identity(state_dim, state_dim);
        H_ = MatrixXd::Zero(measurement_dim, state_dim);
        Q_ = MatrixXd::Identity(state_dim, state_dim);
        R_ = MatrixXd::Identity(measurement_dim, measurement_dim);
    }

    void predict(const VectorXd& control_input) override {
        x_ = F_ * x_;
        if (control_input.size() > 0) {
            x_ += B_ * control_input;
        }
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(const VectorXd& measurement) override {
        VectorXd y = measurement - H_ * x_;
        MatrixXd S = H_ * P_ * H_.transpose() + R_;
        MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        MatrixXd I = MatrixXd::Identity(state_dim_, state_dim_);
        P_ = (I - K * H_) * P_;
    }

    VectorXd getState() const override { return x_; }
    MatrixXd getCovariance() const override { return P_; }
    void setState(const VectorXd& state) override { x_ = state; }
    void setCovariance(const MatrixXd& covariance) override { P_ = covariance; }
    KalmanFilterType getType() const override { return KalmanFilterType::Standard; }

    void setStateTransitionMatrix(const MatrixXd& F) { F_ = F; }
    void setMeasurementMatrix(const MatrixXd& H) { H_ = H; }
    void setProcessNoiseCovariance(const MatrixXd& Q) { Q_ = Q; }
    void setMeasurementNoiseCovariance(const MatrixXd& R) { R_ = R; }
    void setControlMatrix(const MatrixXd& B) { B_ = B; }

private:
    int state_dim_;
    int measurement_dim_;
    VectorXd x_;  // State estimate
    MatrixXd P_;  // State covariance
    MatrixXd F_;  // State transition matrix
    MatrixXd H_;  // Measurement matrix
    MatrixXd Q_;  // Process noise covariance
    MatrixXd R_;  // Measurement noise covariance
    MatrixXd B_;  // Control matrix
};

// Factory function to create appropriate Kalman Filter model
std::unique_ptr<KalmanFilterModel> createKalmanFilterModel(KalmanFilterType type, int state_dim, int measurement_dim) {
    switch (type) {
        case KalmanFilterType::Standard:
            return std::make_unique<StandardKalmanFilterModel>(state_dim, measurement_dim);
        // Implement other types (Extended, Unscented, Ensemble) here
        default:
            throw std::invalid_argument("Unsupported Kalman Filter type");
    }
}

} // namespace advanced_kalman