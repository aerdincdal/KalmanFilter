
#include "advanced_crewai_kalman_filter_system.h"
#include <stdexcept>

namespace advanced_kalman {

    void PredictorAgent::initialize(std::shared_ptr<KalmanFilterModel> model, std::shared_ptr<CommunicationBus> comm_bus) {
        kf_model_ = model;
        comm_bus_ = comm_bus;
    }

    void PredictorAgent::processTask(const TaskData& input, TaskResult& output) {
        if (input.task_type != "predict") {
            throw std::invalid_argument("PredictorAgent received invalid task type");
        }

        VectorXd control_input;
        if (std::holds_alternative<VectorXd>(input.data)) {
            control_input = std::get<VectorXd>(input.data);
        } else {
            throw std::invalid_argument("Invalid control input data type");
        }

        kf_model_->predict(control_input);

        output.result = kf_model_->getState();
        output.confidence = 1.0 / kf_model_->getCovariance().trace();
        output.agent_name = getName();
    }

    void PredictorAgent::communicateResult(const TaskResult& result) {
        comm_bus_->broadcastUpdate({getName(), "State prediction completed", result.result});
    }

    void PredictorAgent::receiveUpdate(const AgentUpdate& update) {
        // Handle updates from other agents if necessary
        // For example, updates from ParameterTunerAgent might require adjusting the model
    }

} // namespace advanced_kalman