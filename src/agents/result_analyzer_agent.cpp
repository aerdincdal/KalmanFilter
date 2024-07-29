
#include "advanced_crewai_kalman_filter_system.h"
#include <stdexcept>
#include <deque>
#include <numeric>
#include <cmath>

namespace advanced_kalman {

class ResultAnalyzerAgent : public AgentInterface {
public:
    void initialize(std::shared_ptr<KalmanFilterModel> model, std::shared_ptr<CommunicationBus> comm_bus) override {
        kf_model_ = model;
        comm_bus_ = comm_bus;
        initializeMetrics();
    }

    std::string getName() const override { return "ResultAnalyzerAgent"; }

    void processTask(const TaskData& input, TaskResult& output) override {
        if (input.task_type != "analyze_results") {
            throw std::invalid_argument("ResultAnalyzerAgent received invalid task type");
        }

        analyzeResults(input);

        output.result = VectorXd::Zero(1);  // Placeholder for analysis result
        output.result(0) = overall_performance_;
        output.confidence = confidence_in_analysis_;
        output.agent_name = getName();
    }

    void communicateResult(const TaskResult& result) override {
        comm_bus_->broadcastUpdate({getName(), "Performance analysis completed", result.result});
    }

    void receiveUpdate(const AgentUpdate& update) override {
        if (update.from_agent == "PredictorAgent" || update.from_agent == "UpdaterAgent") {
            updatePerformanceMetrics(update);
        }
    }

private:
    std::shared_ptr<KalmanFilterModel> kf_model_;
    std::shared_ptr<CommunicationBus> comm_bus_;

    std::deque<double> innovation_history_;
    std::deque<double> estimation_error_history_;
    double overall_performance_;
    double confidence_in_analysis_;

    const size_t MAX_HISTORY_SIZE = 100;

    void initializeMetrics() {
        overall_performance_ = 0.0;
        confidence_in_analysis_ = 0.0;
    }

    void analyzeResults(const TaskData& input) {
        updateOverallPerformance();
        detectAnomalies();
        suggestImprovements();
    }

    void updatePerformanceMetrics(const AgentUpdate& update) {
        if (std::holds_alternative<VectorXd>(update.data)) {
            const VectorXd& state = std::get<VectorXd>(update.data);

            // Calculate innovation (for UpdaterAgent updates)
            if (update.from_agent == "UpdaterAgent") {
                VectorXd predicted_measurement = kf_model_->getMeasurementMatrix() * state;
                VectorXd innovation = kf_model_->getLastMeasurement() - predicted_measurement;
                double innovation_magnitude = innovation.norm();
                updateInnovationHistory(innovation_magnitude);
            }

            // Calculate estimation error (assuming we have access to true state)
            VectorXd true_state = getTrueState();  // This function needs to be implemented
            VectorXd estimation_error = true_state - state;
            double error_magnitude = estimation_error.norm();
            updateEstimationErrorHistory(error_magnitude);
        }
    }

    void updateInnovationHistory(double innovation) {
        innovation_history_.push_back(innovation);
        if (innovation_history_.size() > MAX_HISTORY_SIZE) {
            innovation_history_.pop_front();
        }
    }

    void updateEstimationErrorHistory(double error) {
        estimation_error_history_.push_back(error);
        if (estimation_error_history_.size() > MAX_HISTORY_SIZE) {
            estimation_error_history_.pop_front();
        }
    }

    void updateOverallPerformance() {
        double avg_innovation = std::accumulate(innovation_history_.begin(), innovation_history_.end(), 0.0)
                                / innovation_history_.size();
        double avg_error = std::accumulate(estimation_error_history_.begin(), estimation_error_history_.end(), 0.0)
                           / estimation_error_history_.size();

        // Normalize performance metrics
        double normalized_innovation = 1.0 / (1.0 + avg_innovation);
        double normalized_error = 1.0 / (1.0 + avg_error);

        // Combine metrics (you might want to adjust the weights)
        overall_performance_ = 0.5 * normalized_innovation + 0.5 * normalized_error;

        // Update confidence in analysis
        confidence_in_analysis_ = 1.0 - std::exp(-static_cast<double>(innovation_history_.size()) / MAX_HISTORY_SIZE);
    }

    void detectAnomalies() {
        // Implement anomaly detection logic
        // For example, check for sudden spikes in innovation or estimation error
        // If anomalies are detected, communicate this to other agents
        bool anomaly_detected = false;
        if (!innovation_history_.empty()) {
            double latest_innovation = innovation_history_.back();
            double mean_innovation = std::accumulate(innovation_history_.begin(), innovation_history_.end(), 0.0)
                                     / innovation_history_.size();
            double std_dev_innovation = calculateStandardDeviation(innovation_history_);

            if (std::abs(latest_innovation - mean_innovation) > 3 * std_dev_innovation) {
                anomaly_detected = true;
                comm_bus_->broadcastUpdate({getName(), "Anomaly detected in innovation", VectorXd::Zero(1)});
            }
        }
    }

    void suggestImprovements() {
        // Based on the analysis, suggest improvements to other agents
        if (overall_performance_ < 0.5) {
            // Suggest parameter tuning
            comm_bus_->sendUpdate({"ParameterTunerAgent", "Suggestion for parameter tuning", VectorXd::Zero(1)});
        }
        if (std::accumulate(innovation_history_.begin(), innovation_history_.end(), 0.0) / innovation_history_.size() > 1.0) {
            // Suggest improving the measurement model
            comm_bus_->sendUpdate({"UpdaterAgent", "Suggestion to review measurement model", VectorXd::Zero(1)});
        }
    }

    double calculateStandardDeviation(const std::deque<double>& data) {
        double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0,
            std::plus<>(), [mean](double x, double y) { return (x - mean) * (y - mean); });
        return std::sqrt(sq_sum / data.size());
    }

    VectorXd getTrueState() {
        // This function should return the true state of the system
        // In a real-world scenario, this might not be available, and you'd need to use other methods to evaluate performance
        // For simulation purposes, you might have access to ground truth data
        // This is a placeholder implementation
        return kf_model_->getState();  // In reality, this should be the true state, not the estimated state
    }
};

} // namespace advanced_kalman