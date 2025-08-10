#pragma once

#include <unordered_map>
#include <string>
#include <vector>
#include <set>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "entities/EnvironmentalSensorData.h"

class Aggregator {
public:
    static Aggregator& instance();

    // Регистрация устройства (id — имя с BLE рекламы)
    void addDevice(const std::string& id);

    // Метрики батареи (проценты)
    void addBatteryData(const std::string& id, uint8_t batteryPercent);

    // Мгновенные данные
    void addTemperatureData(const std::string& id, const EnvironmentalSensor::TemperatureSample& s);
    void addHumidityData   (const std::string& id, const EnvironmentalSensor::HumiditySample& s);
    void addPressureData   (const std::string& id, const EnvironmentalSensor::PressureSample& s);
    void addCO2Data        (const std::string& id, const EnvironmentalSensor::CO2Sample& s);
    void addVOCData        (const std::string& id, const EnvironmentalSensor::VOCSample& s);
    void addIAQData        (const std::string& id, const EnvironmentalSensor::IAQSample& s);

    // Сервисная информация (например, сколько реально прочитано из истории)
    void setReadEntries(size_t count);
    size_t getLastReadEntries() const;

    // Опционально можно добавить геттеры под отладку/UI
    bool hasDevice(const std::string& id) const;

private:
    Aggregator();
    ~Aggregator();
    Aggregator(const Aggregator&)            = delete;
    Aggregator& operator=(const Aggregator&) = delete;

private:
    struct DeviceData {
        // Последние известные значения (можно дополнить кольцевыми буферами)
        uint8_t lastBattery{255}; // 255 == unknown

        std::vector<EnvironmentalSensor::TemperatureSample> temperature;
        std::vector<EnvironmentalSensor::HumiditySample>    humidity;
        std::vector<EnvironmentalSensor::PressureSample>    pressure;
        std::vector<EnvironmentalSensor::CO2Sample>         co2;
        std::vector<EnvironmentalSensor::VOCSample>         voc;
        std::vector<EnvironmentalSensor::IAQSample>         iaq;
    };

    // Хранение по device-id
    std::unordered_map<std::string, DeviceData> devices_;

    // Сервисные метрики
    size_t lastReadEntries_{0};

    // Ограничение длины историй (чтобы не росло бесконечно)
    static constexpr size_t MAX_POINTS_PER_STREAM = 1024;

    // Мьютекс
    SemaphoreHandle_t mutex_;
    void lock() const;
    void unlock() const;

    template<typename T>
    static void pushBounded(std::vector<T>& vec, const T& v, size_t maxN) {
        if (vec.size() >= maxN) {
            // простейшая политика — удаляем старейший
            vec.erase(vec.begin());
        }
        vec.emplace_back(v);
    }
};
