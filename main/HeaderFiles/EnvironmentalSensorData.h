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
    Source source{Source::UNKNOWN};

    inline void set_source(Source s) { source = s; }
};

template<typename T>
struct Sample {
    uint32_t timestamp{0}; 
    Flags    flags{};
    T        value{};    
};

using TemperatureSample = Sample<float>;  // °C
using HumiditySample    = Sample<float>; // %RH
using PressureSample    = Sample<float>;  // Па/гПа 
using CO2Sample         = Sample<float>;  // ppm
} 
