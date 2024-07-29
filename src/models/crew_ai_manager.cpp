#include "ultimate_multi_vehicle_kalman_system.h"
#include <algorithm>
#include <stdexcept>

namespace ultimate_kalman {

CrewAIManager::CrewAIManager(KalmanFilterType filter_type, int state_dim, int measurement_dim)
    : task_scheduler_(std::make_unique<TaskScheduler>(std::thread::hardware_concurrency())),
      comm_bus_(std::make_shared<CommunicationBus>()),
      ai_decision_maker_(std::make_unique<AIDecisionMaker>()) {
    kf_model_ = createAdvancedKalmanFilterModel(filter_type, state_dim, measurement_dim);
}

void CrewAIManager::initializeCrew() {
    createAgents();
    for (auto& agent : agents_) {
        agent->initialize(kf_model_, comm_bus_);
        comm_bus_->registerAgent(agent->getName(), agent);
    }
    ai_decision_maker_->initializeModel(state_dim_, measurement_dim_);
}

void CrewAIManager::createAgents() {
    agents_.push_back(std::make_shared<PredictorAgent>());
    agents_.push_back(std::make_shared<UpdaterAgent>());
    agents_.push_back(std::make_shared<DataPreprocessorAgent>());
    agents_.push_back(std::make_shared<ParameterTunerAgent>());
    agents_.push_back(std::make_shared<ResultAnalyzerAgent>());
}

void CrewAIManager::processData(const std::vector<SensorData>& sensor_data) {
    assignTasks(sensor_data);
    task_scheduler_->waitForCompletion();
    gatherResults();
    updateAIModel();
}

void CrewAIManager::assignTasks(const std::vector<SensorData>& sensor_data) {
    for (const auto& data : sensor_data) {
        task_scheduler_->addTask([this, data]() {
            TaskData task_data{data, kf_model_->getType(), "process_sensor_data"};
            TaskResult result;
            agents_[0]->processTask(task_data, result);  // Assume first agent is DataPreprocessorAgent
            comm_bus_->broadcastUpdate({result.agent_name, "New data processed", result.result});
        });
    }
}

void CrewAIManager::gatherResults() {
    final_state_ = agents_[1]->getLatestState();  // Assume second agent is PredictorAgent
    final_covariance_ = agents_[1]->getLatestCovariance();

    auto analysis = agents_[4]->getAnalysis();  // Assume fifth agent is ResultAnalyzerAgent
    updateSystemPerformance(analysis);
}

void CrewAIManager::updateAIModel() {
    SystemState current_state{final_state_, final_covariance_, system_performance_};
    ai_decision_maker_->updateModel(current_state);
}

void CrewAIManager::updateSystemPerformance(const PerformanceMetrics& metrics) {
    system_performance_.accuracy = metrics.estimation_accuracy;
    system_performance_.reliability = metrics.system_reliability;
    // Update other performance indicators
}

VectorXd CrewAIManager::getFinalState() const {
    return final_state_;
}

MatrixXd CrewAIManager::getFinalCovariance() const {
    return final_covariance_;
}

void CrewAIManager::optimizePerformance() {
    auto decision = ai_decision_maker_->makeDecision(SystemState{final_state_, final_covariance_, system_performance_});
    applyOptimizationDecision(decision);
}

void CrewAIManager::applyOptimizationDecision(const AIDecision& decision) {
    if (decision.should_adjust_parameters) {
        dynamic_cast<ParameterTunerAgent&>(*agents_[3]).tuneParameters(decision.parameter_adjustments);
    }
    if (decision.should_reconfigure_agents) {
        reconfigureAgents(decision.agent_configuration);
    }
    // Apply other optimization decisions
}

void CrewAIManager::reconfigureAgents(const AgentConfiguration& config) {
    // Implement agent reconfiguration logic
}

} // namespace ultimate_kalman