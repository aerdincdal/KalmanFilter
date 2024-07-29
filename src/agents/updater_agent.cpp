
#include "advanced_crewai_kalman_filter_system.h"
#include <stdexcept>

namespace advanced_kalman {

    void UpdaterAgent::initialize(std::shared_ptr<KalmanFilterModel> model, std::shared_ptr<CommunicationBus> comm_bus) {
        kf_model_ = model;
        comm_bus_ = comm_bus;
    }

    void UpdaterAgent::processTask(const TaskData& input, TaskResult& output) {
        if (input.task_type != "update") {
            throw std::invalid_argument("UpdaterAgent received invalid task type");
        }

        VectorXd measurement;
        if (std::holds_alternative<SensorData>(input.data)) {
            measurement = std::get<SensorData>(input.data).measurement;
        } else {
            throw std::invalid_argument("Invalid measurement data type");
        }

        kf_model_->update(measurement);

        output.result = kf_model_->getState();
        output.confidence = 1.0 / kf_model_->getCovariance().trace();
        output.agent_name = getName();
    }

    void UpdaterAgent::communicateResult(const TaskResult& result) {
        comm_bus_->broadcastUpdate({getName(), "State update completed", result.result});
    }

    void UpdaterAgent::receiveUpdate(const AgentUpdate& update) {
        if (update.from_agent == "ParameterTunerAgent") {
            // Handle parameter updates
            if (std::holds_alternative<MatrixXd>(update.data)) {
                MatrixXd new_measurement_noise = std::get<MatrixXd>(update.data);
                // Update measurement noise in the model
                // Note: This assumes KalmanFilterModel has a method to set measurement noise
                // kf_model_->setMeasurementNoiseCovariance(new_measurement_noise);
            }
        }
    }

} // namespace advanced_kalman