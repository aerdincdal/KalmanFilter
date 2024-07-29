
#include "advanced_crewai_kalman_filter_system.h"
#include <stdexcept>
#include <cmath>

namespace advanced_kalman {

void DataPreprocessorAgent::initialize(std::shared_ptr<KalmanFilterModel> model, std::shared_ptr<CommunicationBus> comm_bus) {
    comm_bus_ = comm_bus;
    // DataPreprocessorAgent doesn't need direct access to the KalmanFilterModel
}

void DataPreprocessorAgent::processTask(const TaskData& input, TaskResult& output) {
    if (input.task_type != "preprocess") {
        throw std::invalid_argument("DataPreprocessorAgent received invalid task type");
    }

    SensorData raw_data;
    if (std::holds_alternative<SensorData>(input.data)) {
        raw_data = std::get<SensorData>(input.data);
    } else {
        throw std::invalid_argument("Invalid input data type for preprocessing");
    }

    // Perform preprocessing steps
    VectorXd processed_measurement = preprocessMeasurement(raw_data.measurement);

    output.result = processed_measurement;
    output.confidence = calculateConfidence(raw_data.measurement, processed_measurement);
    output.agent_name = getName();
}

void DataPreprocessorAgent::communicateResult(const TaskResult& result) {
    comm_bus_->broadcastUpdate({getName(), "Data preprocessing completed", result.result});
}

void DataPreprocessorAgent::receiveUpdate(const AgentUpdate& update) {
    // DataPreprocessorAgent might receive updates about new preprocessing techniques
    // or changes in data quality expectations
}

VectorXd DataPreprocessorAgent::preprocessMeasurement(const VectorXd& raw_measurement) {
    // Implement preprocessing steps here. For example:
    // 1. Remove outliers
    // 2. Handle missing data
    // 3. Normalize or scale data
    // 4. Apply filters (e.g., low-pass filter)

    VectorXd processed = raw_measurement;  // Start with a copy of the raw data

    // Example: Simple moving average filter
    const int window_size = 5;
    static std::vector<VectorXd> history(window_size, VectorXd::Zero(raw_measurement.size()));

    history.push_back(raw_measurement);
    history.erase(history.begin());

    processed = VectorXd::Zero(raw_measurement.size());
    for (const auto& hist : history) {
        processed += hist;
    }
    processed /= window_size;

    return processed;
}

double DataPreprocessorAgent::calculateConfidence(const VectorXd& raw, const VectorXd& processed) {
    // Calculate confidence based on the difference between raw and processed data
    double diff = (raw - processed).norm();
    return 1.0 / (1.0 + diff);  // Confidence decreases as difference increases
}

} // namespace advanced_kalman