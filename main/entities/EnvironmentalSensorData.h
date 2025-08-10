#pragma once

#include <cstdint>
#include <string>

namespace EnvironmentalSensor {

enum class Source : uint8_t {
    UNKNOWN = 0,
    BLE     = 1,
    WIFI    = 2,
    LOCAL   = 3
};

struct Flags {
    // битовые флаги, но храним как полями для наглядности
    bool   history{false};
    Source source{Source::UNKNOWN};

    inline void set_history(bool v) { history = v; }
    inline void set_source(Source s) { source = s; }
};

template<typename T>
struct Sample {
    uint32_t timestamp{0};  // сек unix
    Flags    flags{};
    T        value{};       // физическая величина (в SI или как вы передаёте)
};

// Отдельные алиасы для наглядности
using TemperatureSample = Sample<float>;  // °C
using HumiditySample    = Sample<float>; // %RH
using PressureSample    = Sample<float>;  // Па/гПа — у вас уже подаётся float(pressure)
using CO2Sample         = Sample<float>;  // ppm
using VOCSample         = Sample<float>;  // индекс / ppb — по вашему источнику
using IAQSample         = Sample<float>;  // индекс

} // namespace EnvironmentalSensor
