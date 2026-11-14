#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

struct ImuAcceleration {
    float x, y, z; // m/s^2, including gravity, sensor coordinate frame
    int64_t received_us;
};

// Single sensor / single SH-2 instance. Initialization precedes other I2C tasks.
// Polls either 0x4A or 0x4B; no INT or RESET GPIO is required by this adapter.
esp_err_t bno086_start(i2c_master_bus_handle_t bus);
// False before the first report, after reset, or if data is older than 500 ms.
bool bno086_latest(ImuAcceleration &sample);
