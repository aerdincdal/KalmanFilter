#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <Eigen/Dense>
#include <string>

namespace ultimate_kalman {

    enum class SensorType {
        GPS,
        IMU,
        Camera,
        Lidar,
        Radar
    };

    struct SensorData {
        // Define sensor data properties
    };

    class SensorInterface {
    public:
        virtual ~SensorInterface() = default;
        virtual SensorData getData() const = 0;
        virtual SensorType getType() const = 0;
        virtual int getMeasurementDimension() const = 0;
        virtual double getReliability() const = 0;
        virtual void calibrate() = 0;
        virtual bool isMalfunctioning() const = 0;
        virtual void applyNoiseReduction() = 0;

        std::string id;
        SensorType type;
    };

} // namespace ultimate_kalman

#endif // SENSOR_INTERFACE_H