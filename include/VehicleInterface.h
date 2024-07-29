#ifndef VEHICLE_INTERFACE_H
#define VEHICLE_INTERFACE_H

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace ultimate_kalman {

    enum class VehicleType {
        Ground,
        Aerial,
        Maritime,
        Space
    };

    struct VehicleState {
        // Define vehicle state properties
    };

    struct CollisionData {
        // Define collision data properties
    };

    class VehicleInterface {
    public:
        virtual ~VehicleInterface() = default;
        virtual void updateState(double time_step) = 0;
        virtual VehicleState getState() const = 0;
        virtual VehicleType getType() const = 0;
        virtual int getStateDimension() const = 0;
        virtual void applyControl(const Eigen::VectorXd& control_input) = 0;
        virtual void handleCollision(const CollisionData& collision) = 0;
        virtual void performSelfDiagnostics() = 0;
        virtual double getRemainingFuel() const = 0;
        virtual std::vector<std::string> getEquipmentStatus() const = 0;

        std::string id;
        VehicleType type;
    };

} // namespace ultimate_kalman

#endif // VEHICLE_INTERFACE_H