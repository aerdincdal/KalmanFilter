#ifndef AI_DECISION_MAKER_H
#define AI_DECISION_MAKER_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "system_state.h"
#include "ai_decision.h"

namespace ultimate_kalman {

    class NeuralNetwork;  // Forward declaration

    class AIDecisionMaker {
    public:
        AIDecisionMaker();
        ~AIDecisionMaker();

        void initializeModel(int state_dim, int measurement_dim);
        void updateModel(const SystemState& current_state);
        AIDecision makeDecision(const SystemState& current_state);
        void trainModel(const std::vector<TrainingData>& training_data);

    private:
        std::unique_ptr<NeuralNetwork> neural_network_;
        int state_dim_;
        int measurement_dim_;

        Eigen::VectorXd preprocessState(const SystemState& state) const;
        AIDecision interpretOutput(const Eigen::VectorXd& nn_output) const;
        void validateDecision(AIDecision& decision) const;
        double evaluateDecisionQuality(const AIDecision& decision, const SystemState& state) const;
        void updateModelWeights(const AIDecision& decision, double reward);
    };

} // namespace ultimate_kalman

#endif // AI_DECISION_MAKER_H