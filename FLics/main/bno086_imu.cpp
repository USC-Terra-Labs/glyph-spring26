#include "bno086_imu.h"

#include <cmath>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
extern "C" {
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
}

namespace {
constexpr char TAG[] = "bno086";
constexpr int I2C_TIMEOUT_MS = 100;
constexpr int64_t MAX_SAMPLE_AGE_US = 500000;
constexpr uint32_t REPORT_INTERVAL_US = 20000; // 50 Hz, no batching
i2c_master_dev_handle_t device = nullptr;
QueueHandle_t samples = nullptr;
bool running = false;
bool reset_pending = false; // Only accessed by the SH-2 owner task.
sh2_Hal_t hal = {};

int hal_open(sh2_Hal_t *) {
    // SHTP executable channel, reset command. Soft reset preserves four-wire I2C wiring.
    const uint8_t reset[] = {5, 0, 1, 0, 1};
    for (int attempt = 0; attempt < 5; ++attempt) {
        if (i2c_master_transmit(device, reset, sizeof(reset), I2C_TIMEOUT_MS) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(300));
            return SH2_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    return SH2_ERR_IO;
}

void hal_close(sh2_Hal_t *) {} // Device lifetime is managed by bno086_start.

int hal_read(sh2_Hal_t *, uint8_t *buffer, unsigned capacity, uint32_t *timestamp) {
    uint8_t header[4];
    *timestamp = static_cast<uint32_t>(esp_timer_get_time());
    if (i2c_master_receive(device, header, sizeof(header), I2C_TIMEOUT_MS) != ESP_OK) {
        vTaskDelay(1);
        return 0;
    }
    const unsigned length = (header[0] | (unsigned(header[1]) << 8)) & 0x7fff;
    if (length < 4 || length > capacity) {
        // Empty/invalid transport header: do not pass it to the SH-2 decoder.
        vTaskDelay(1);
        return 0;
    }
    // BNO08x repeats its four-byte SHTP header on the following I2C read.
    // ESP-IDF supports receiving the entire transfer, without Wire-style chunking.
    if (i2c_master_receive(device, buffer, length, I2C_TIMEOUT_MS) != ESP_OK) {
        vTaskDelay(1);
        return 0;
    }
    return static_cast<int>(length);
}

int hal_write(sh2_Hal_t *, uint8_t *buffer, unsigned length) {
    if (length > SH2_HAL_MAX_TRANSFER_OUT) return SH2_ERR_BAD_PARAM;
    if (i2c_master_transmit(device, buffer, length, I2C_TIMEOUT_MS) == ESP_OK)
        return static_cast<int>(length);
    vTaskDelay(1);
    return 0;
}

uint32_t hal_time(sh2_Hal_t *) {
    return static_cast<uint32_t>(esp_timer_get_time());
}

void on_event(void *, sh2_AsyncEvent_t *event) {
    if (event->eventId == SH2_RESET) {
        reset_pending = true;
        xQueueReset(samples);
    }
}

void on_sensor(void *, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value = {};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK ||
        value.sensorId != SH2_ACCELEROMETER) return;
    const auto &a = value.un.accelerometer;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z)) return;
    ImuAcceleration sample = {a.x, a.y, a.z, esp_timer_get_time()};
    xQueueOverwrite(samples, &sample);
}

int enable_accelerometer() {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = REPORT_INTERVAL_US;
    return sh2_setSensorConfig(SH2_ACCELEROMETER, &config);
}

void service_task(void *) {
    int64_t last_retry = 0;
    int64_t last_warning = 0;
    while (true) {
        // The CEVA library is not reentrant: this task owns all runtime SH-2 calls.
        sh2_service();
        const int64_t now = esp_timer_get_time();
        if (reset_pending && now - last_retry >= 1000000) {
            last_retry = now;
            reset_pending = false;
            if (enable_accelerometer() != SH2_OK) {
                reset_pending = true;
                ESP_LOGW(TAG, "Could not restore accelerometer reports after reset");
            } else {
                ESP_LOGI(TAG, "Accelerometer reports restored after sensor reset");
            }
        }
        ImuAcceleration sample;
        if (!bno086_latest(sample) && now - last_warning >= 5000000) {
            last_warning = now;
            ESP_LOGW(TAG, "No fresh acceleration; check IMU power/I2C wiring");
        }
        vTaskDelay(1);
    }
}
} // namespace

esp_err_t bno086_start(i2c_master_bus_handle_t bus) {
    if (running) return ESP_ERR_INVALID_STATE;
    if (!bus) return ESP_ERR_INVALID_ARG;
    if (!samples) samples = xQueueCreate(1, sizeof(ImuAcceleration));
    if (!samples) return ESP_ERR_NO_MEM;
    xQueueReset(samples);
    hal.open = hal_open;
    hal.close = hal_close;
    hal.read = hal_read;
    hal.write = hal_write;
    hal.getTimeUs = hal_time;

    const uint8_t addresses[] = {0x4A, 0x4B};
    for (uint8_t address : addresses) {
        if (i2c_master_probe(bus, address, I2C_TIMEOUT_MS) != ESP_OK) continue;
        i2c_device_config_t config = {};
        config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        config.device_address = address;
        config.scl_speed_hz = 100000; // Conservative starting speed for BNO08x clock stretching.
        config.scl_wait_us = 0xFFFF;
        esp_err_t err = i2c_master_bus_add_device(bus, &config, &device);
        if (err != ESP_OK) return err;
        reset_pending = false;
        int status = sh2_open(&hal, on_event, nullptr);
        sh2_ProductIds_t ids = {};
        if (status == SH2_OK) status = sh2_getProdIds(&ids);
        if (status == SH2_OK && ids.numEntries == 0) status = SH2_ERR;
        if (status == SH2_OK) status = sh2_setSensorCallback(on_sensor, nullptr);
        if (status == SH2_OK) {
            reset_pending = false;
            status = enable_accelerometer();
        }
        // Validate an actual report, not merely an ACK or successful config write.
        const int64_t deadline = esp_timer_get_time() + 2000000;
        ImuAcceleration sample;
        while (status == SH2_OK && !bno086_latest(sample) && esp_timer_get_time() < deadline) {
            sh2_service();
            vTaskDelay(1);
        }
        if (status == SH2_OK && bno086_latest(sample)) {
            if (xTaskCreate(service_task, "bno086", 6144, nullptr, 6, nullptr) == pdPASS) {
                running = true;
                ESP_LOGI(TAG, "SH-2 IMU at 0x%02x, product %lu, firmware %u.%u; acceleration 50 Hz",
                         address, (unsigned long)ids.entry[0].swPartNumber,
                         ids.entry[0].swVersionMajor, ids.entry[0].swVersionMinor);
                ESP_LOGI(TAG, "First acceleration: x=%.3f y=%.3f z=%.3f m/s^2",
                         sample.x, sample.y, sample.z);
                return ESP_OK;
            }
            status = SH2_ERR;
        }
        ESP_LOGW(TAG, "No usable SH-2 acceleration at 0x%02x (status %d)", address, status);
        sh2_close();
        i2c_master_bus_rm_device(device);
        device = nullptr;
        xQueueReset(samples);
    }
    return ESP_ERR_NOT_FOUND;
}

bool bno086_latest(ImuAcceleration &sample) {
    if (!samples || xQueuePeek(samples, &sample, 0) != pdTRUE) return false;
    return esp_timer_get_time() - sample.received_us <= MAX_SAMPLE_AGE_US;
}
