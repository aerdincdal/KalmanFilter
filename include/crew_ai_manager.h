#ifndef CREW_AI_MANAGER_H
#define CREW_AI_MANAGER_H

#include <vector>
#include <memory>
#include "advanced_kalman_filter.h"
#include "agent_interface.h"
#include "task_scheduler.h"
#include "communication_bus.h"
#include "ai_decision_maker.h"
#include "sensor_data.h"
#include "system_performance.h"

namespace ultimate_kalman {

    class CrewAIManager {
    public:
        CrewAIManager(KalmanFilterType filter_type, int state_dim, int measurement_dim);
        ~CrewAIManager() = default;

        void initializeCrew();
        void processData(const std::vector<SensorData>& sensor_data);
        Eigen::VectorXd getFinalState() const;
        Eigen::MatrixXd getFinalCovariance() const;
        void optimizePerformance();

    private:
        std::unique_ptr<AdvancedKalmanFilter> kf_model_;
        std::vector<std::shared_ptr<AgentInterface>> agents_;
        std::unique_ptr<TaskScheduler> task_scheduler_;
        std::shared_ptr<CommunicationBus> comm_bus_;
        std::unique_ptr<AIDecisionMaker> ai_decision_maker_;

        Eigen::VectorXd final_state_;
        Eigen::MatrixXd final_covariance_;
        SystemPerformance system_performance_;

        int state_dim_;
        int measurement_dim_;

        void createAgents();
        void assignTasks(const std::vector<SensorData>& sensor_data);
        void gatherResults();
        void updateAIModel();
        void updateSystemPerformance(const PerformanceMetrics& metrics);
        void applyOptimizationDecision(const AIDecision& decision);
        void reconfigureAgents(const AgentConfiguration& config);
    };

} // namespace ultimate_kalman

#endif // CREW_AI_MANAGER_H