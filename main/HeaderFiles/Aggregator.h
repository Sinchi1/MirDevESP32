#pragma once

#include <unordered_map>
#include <string>
#include <vector>
#include <set>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "EnvironmentalSensorData.h"

class Aggregator {
public:
    static Aggregator& instance();

    void addDevice(const std::string& id);

    void addBatteryData(const std::string& id, uint8_t batteryPercent);

    void addTemperatureData(const std::string& id, const EnvironmentalSensor::TemperatureSample& s);
    void addHumidityData   (const std::string& id, const EnvironmentalSensor::HumiditySample& s);
    void addPressureData   (const std::string& id, const EnvironmentalSensor::PressureSample& s);
    void addCO2Data        (const std::string& id, const EnvironmentalSensor::CO2Sample& s);

    void setReadEntries(size_t count);
    size_t getLastReadEntries() const;

    bool hasDevice(const std::string& id) const;

private:
    Aggregator();
    ~Aggregator();
    Aggregator(const Aggregator&)            = delete;
    Aggregator& operator=(const Aggregator&) = delete;

private:
    struct DeviceData {
        uint8_t lastBattery{255}; 

        std::vector<EnvironmentalSensor::TemperatureSample> temperature;
        std::vector<EnvironmentalSensor::HumiditySample>    humidity;
        std::vector<EnvironmentalSensor::PressureSample>    pressure;
        std::vector<EnvironmentalSensor::CO2Sample>         co2;
    };

    std::unordered_map<std::string, DeviceData> devices_;

    size_t lastReadEntries_{0};

    static constexpr size_t MAX_POINTS_PER_STREAM = 1024;

    SemaphoreHandle_t mutex_;
    void lock() const;
    void unlock() const;

    template<typename T>
    static void pushBounded(std::vector<T>& vec, const T& v, size_t maxN) {
        if (vec.size() >= maxN) {
            vec.erase(vec.begin());
        }
        vec.emplace_back(v);
    }
};
