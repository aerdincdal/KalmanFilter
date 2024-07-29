// File: src/advanced_kalman_filter.cpp

#include "ultimate_multi_vehicle_kalman_system.h"
#include <random>
#include <cmath>

namespace ultimate_kalman {

AdvancedKalmanFilter::AdvancedKalmanFilter(FilterType type, int state_dim, int measurement_dim)
    : type_(type), x_(state_dim), P_(state_dim, state_dim),
      F_(state_dim, state_dim), H_(measurement_dim, state_dim),
      Q_(state_dim, state_dim), R_(measurement_dim, measurement_dim) {

    x_.setZero();
    P_.setIdentity();
    F_.setIdentity();
    H_.setZero();
    Q_.setIdentity();
    R_.setIdentity();

    if (type_ == FilterType::UKF) {
        initializeUKF(state_dim);
    } else if (type_ == FilterType::PF) {
        initializePF(state_dim);
    }
}

void AdvancedKalmanFilter::predict(const Eigen::VectorXd& control_input) {
    switch (type_) {
        case FilterType::EKF:
            x_ = F_ * x_ + control_input;
            P_ = F_ * P_ * F_.transpose() + Q_;
            break;
        case FilterType::UKF:
            predictUKF(control_input);
            break;
        case FilterType::PF:
            predictPF(control_input);
            break;
    }
}

void AdvancedKalmanFilter::update(const Eigen::VectorXd& measurement) {
    switch (type_) {
        case FilterType::EKF:
            performEKFUpdate(measurement);
            break;
        case FilterType::UKF:
            performUKFUpdate(measurement);
            break;
        case FilterType::PF:
            performPFUpdate(measurement);
            break;
    }
}

void AdvancedKalmanFilter::performEKFUpdate(const Eigen::VectorXd& measurement) {
    Eigen::VectorXd y = measurement - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

void AdvancedKalmanFilter::performUKFUpdate(const Eigen::VectorXd& measurement) {
    generateSigmaPoints();

    // Transform sigma points through measurement model
    std::vector<Eigen::VectorXd> transformed_sigmas;
    for (const auto& sigma : sigma_points_) {
        transformed_sigmas.push_back(H_ * sigma);
    }

    // Calculate mean and covariance of predicted measurement
    Eigen::VectorXd y_mean = Eigen::VectorXd::Zero(measurement.size());
    for (const auto& y : transformed_sigmas) {
        y_mean += y;
    }
    y_mean /= transformed_sigmas.size();

    Eigen::MatrixXd Pyy = Eigen::MatrixXd::Zero(measurement.size(), measurement.size());
    Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(x_.size(), measurement.size());
    for (size_t i = 0; i < transformed_sigmas.size(); ++i) {
        Eigen::VectorXd diff_y = transformed_sigmas[i] - y_mean;
        Eigen::VectorXd diff_x = sigma_points_[i] - x_;
        Pyy += diff_y * diff_y.transpose();
        Pxy += diff_x * diff_y.transpose();
    }
    Pyy /= transformed_sigmas.size();
    Pxy /= sigma_points_.size();

    // Update state and covariance
    Eigen::MatrixXd K = Pxy * Pyy.inverse();
    x_ = x_ + K * (measurement - y_mean);
    P_ = P_ - K * Pyy * K.transpose();
}

void AdvancedKalmanFilter::performPFUpdate(const Eigen::VectorXd& measurement) {
    // Update particle weights based on measurement likelihood
    for (size_t i = 0; i < particle_weights_.size(); ++i) {
        Eigen::VectorXd innovation = measurement - H_ * sigma_points_[i];
        double likelihood = std::exp(-0.5 * innovation.transpose() * R_.inverse() * innovation);
        particle_weights_[i] *= likelihood;
    }

    // Normalize weights
    double sum_weights = std::accumulate(particle_weights_.begin(), particle_weights_.end(), 0.0);
    for (auto& weight : particle_weights_) {
        weight /= sum_weights;
    }

    // Compute effective number of particles
    double neff = 1.0 / particle_weights_.squaredNorm();

    // Resample if necessary
    if (neff < particle_weights_.size() / 2) {
        resampleParticles();
    }

    // Update state estimate
    x_.setZero();
    for (size_t i = 0; i < sigma_points_.size(); ++i) {
        x_ += particle_weights_[i] * sigma_points_[i];
    }

    // Update covariance estimate
    P_.setZero();
    for (size_t i = 0; i < sigma_points_.size(); ++i) {
        Eigen::VectorXd diff = sigma_points_[i] - x_;
        P_ += particle_weights_[i] * (diff * diff.transpose());
    }
}

void AdvancedKalmanFilter::generateSigmaPoints() {
    int n = x_.size();
    double lambda = 3 - n;  // Scaling parameter

    sigma_points_.clear();
    sigma_points_.push_back(x_);

    Eigen::MatrixXd L = P_.llt().matrixL();
    for (int i = 0; i < n; ++i) {
        sigma_points_.push_back(x_ + std::sqrt(n + lambda) * L.col(i));
        sigma_points_.push_back(x_ - std::sqrt(n + lambda) * L.col(i));
    }
}

void AdvancedKalmanFilter::resampleParticles() {
    std::vector<Eigen::VectorXd> new_particles;
    std::vector<double> new_weights;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (size_t i = 0; i < sigma_points_.size(); ++i) {
        double u = dis(gen);
        double cumsum = 0.0;
        for (size_t j = 0; j < sigma_points_.size(); ++j) {
            cumsum += particle_weights_[j];
            if (u < cumsum) {
                new_particles.push_back(sigma_points_[j]);
                new_weights.push_back(1.0 / sigma_points_.size());
                break;
            }
        }
    }

    sigma_points_ = std::move(new_particles);
    particle_weights_ = std::move(new_weights);
}

void AdvancedKalmanFilter::adaptFilterParameters(const AdaptiveFilteringData& adaptive_data) {
    // Implement adaptive Q and R estimation
    // This is a simple implementation and can be expanded based on specific requirements
    Q_ = adaptive_data.estimated_process_noise;
    R_ = adaptive_data.estimated_measurement_noise;
}

double AdvancedKalmanFilter::getInnovationLikelihood() const {
    Eigen::VectorXd innovation = H_ * x_;  // Assuming last measurement is stored
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    return -0.5 * (innovation.transpose() * S.inverse() * innovation + std::log(S.determinant()));
}

void AdvancedKalmanFilter::performConsistencyCheck() {
    // Implement chi-square test for filter consistency
    Eigen::VectorXd innovation = H_ * x_;  // Assuming last measurement is stored
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    double nis = innovation.transpose() * S.inverse() * innovation;

    // Check if NIS is within acceptable bounds (e.g., 95% confidence interval)
    int dof = innovation.size();
// ... (önceki kod devam ediyor)

    double chi_square_lower = boost::math::chi_squared_quantile(0.025, dof);
    double chi_square_upper = boost::math::chi_squared_quantile(0.975, dof);

    if (nis < chi_square_lower || nis > chi_square_upper) {
        // Filter may be inconsistent, trigger adaptive mechanism
        adaptFilterParameters(estimateAdaptiveParameters());
    }
}

void AdvancedKalmanFilter::handleMissingMeasurements(const std::vector<bool>& measurement_mask) {
    // Adjust H and R matrices based on available measurements
    Eigen::MatrixXd H_adjusted = H_;
    Eigen::MatrixXd R_adjusted = R_;
    int available_measurements = 0;

    for (size_t i = 0; i < measurement_mask.size(); ++i) {
        if (!measurement_mask[i]) {
            H_adjusted.row(i).setZero();
            R_adjusted.row(i).setZero();
            R_adjusted.col(i).setZero();
        } else {
            ++available_measurements;
        }
    }

    // Perform update with adjusted matrices
    if (available_measurements > 0) {
        Eigen::MatrixXd S = H_adjusted * P_ * H_adjusted.transpose() + R_adjusted;
        Eigen::MatrixXd K = P_ * H_adjusted.transpose() * S.inverse();
        x_ = x_ + K * (getMeasurementWithMask(measurement_mask) - H_adjusted * x_);
        P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H_adjusted) * P_;
    }
    // If no measurements are available, only prediction step is performed
}

Eigen::VectorXd AdvancedKalmanFilter::getMeasurementWithMask(const std::vector<bool>& measurement_mask) const {
    // This function should return the actual measurement vector with unavailable measurements set to 0
    // Implement based on how measurements are stored/provided in your system
    return Eigen::VectorXd::Zero(measurement_mask.size());  // Placeholder
}

AdaptiveFilteringData AdvancedKalmanFilter::estimateAdaptiveParameters() {
    AdaptiveFilteringData adaptive_data;

    // Estimate Q using innovation sequence
    static std::vector<Eigen::VectorXd> innovation_sequence;
    Eigen::VectorXd innovation = H_ * x_;  // Assuming last measurement is stored
    innovation_sequence.push_back(innovation);

    if (innovation_sequence.size() > 20) {  // Use a window of past innovations
        Eigen::MatrixXd Q_est = Eigen::MatrixXd::Zero(Q_.rows(), Q_.cols());
        for (const auto& inn : innovation_sequence) {
            Q_est += inn * inn.transpose();
        }
        Q_est /= innovation_sequence.size();
        Q_est -= H_ * P_ * H_.transpose() + R_;
        adaptive_data.estimated_process_noise = Q_est.cwiseMax(Eigen::MatrixXd::Zero(Q_.rows(), Q_.cols()));  // Ensure positive semi-definite

        innovation_sequence.erase(innovation_sequence.begin());  // Remove oldest innovation
    } else {
        adaptive_data.estimated_process_noise = Q_;  // Use current Q if not enough data
    }

    // Estimate R using residual sequence
    static std::vector<Eigen::VectorXd> residual_sequence;
    Eigen::VectorXd residual = innovation - H_ * (F_ * x_);
    residual_sequence.push_back(residual);

    if (residual_sequence.size() > 20) {
        Eigen::MatrixXd R_est = Eigen::MatrixXd::Zero(R_.rows(), R_.cols());
        for (const auto& res : residual_sequence) {
            R_est += res * res.transpose();
        }
        R_est /= residual_sequence.size();
        adaptive_data.estimated_measurement_noise = R_est.cwiseMax(Eigen::MatrixXd::Zero(R_.rows(), R_.cols()));  // Ensure positive semi-definite

        residual_sequence.erase(residual_sequence.begin());  // Remove oldest residual
    } else {
        adaptive_data.estimated_measurement_noise = R_;  // Use current R if not enough data
    }

    return adaptive_data;
}

// Additional helper methods

void AdvancedKalmanFilter::initializeUKF(int state_dim) {
    // Initialize UKF-specific parameters
    double alpha = 1e-3;
    double beta = 2.0;
    double kappa = 0.0;
    double lambda = alpha * alpha * (state_dim + kappa) - state_dim;

    // Precompute weights
    double weight_mean_0 = lambda / (state_dim + lambda);
    double weight_cov_0 = weight_mean_0 + (1 - alpha * alpha + beta);
    double weight_i = 1 / (2 * (state_dim + lambda));

    sigma_points_.reserve(2 * state_dim + 1);
    particle_weights_.resize(2 * state_dim + 1);

    particle_weights_[0] = weight_mean_0;
    for (int i = 1; i < 2 * state_dim + 1; ++i) {
        particle_weights_[i] = weight_i;
    }
}

void AdvancedKalmanFilter::initializePF(int state_dim) {
    // Initialize Particle Filter parameters
    const int num_particles = 1000;  // Can be adjusted based on state dimension and required accuracy
    sigma_points_.reserve(num_particles);
    particle_weights_.resize(num_particles, 1.0 / num_particles);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis(0, 1);

    for (int i = 0; i < num_particles; ++i) {
        Eigen::VectorXd particle = x_ + P_.llt().matrixL() * Eigen::VectorXd::NullaryExpr(state_dim, [&]() { return dis(gen); });
        sigma_points_.push_back(particle);
    }
}

void AdvancedKalmanFilter::predictUKF(const Eigen::VectorXd& control_input) {
    generateSigmaPoints();

    // Propagate sigma points
    for (auto& sigma : sigma_points_) {
        sigma = F_ * sigma + control_input;
    }

    // Recombine sigma points
    x_.setZero();
    P_.setZero();
    for (size_t i = 0; i < sigma_points_.size(); ++i) {
        x_ += particle_weights_[i] * sigma_points_[i];
    }
    for (size_t i = 0; i < sigma_points_.size(); ++i) {
        Eigen::VectorXd diff = sigma_points_[i] - x_;
        P_ += particle_weights_[i] * (diff * diff.transpose());
    }
    P_ += Q_;
}

void AdvancedKalmanFilter::predictPF(const Eigen::VectorXd& control_input) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis(0, 1);

    for (auto& particle : sigma_points_) {
        Eigen::VectorXd noise = Q_.llt().matrixL() * Eigen::VectorXd::NullaryExpr(x_.size(), [&]() { return dis(gen); });
        particle = F_ * particle + control_input + noise;
    }
}

} // namespace ultimate_kalman