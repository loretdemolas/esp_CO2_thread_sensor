/*
 * scd4x.c - port of legacy driver to ESP-IDF I2C v2
 *
 * Implements the full command set you provided.
 */

#include "scd4x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "scd4x_v2";

/* Commands from datasheet */
#define CMD_START_PERIODIC_MEASUREMENT             (0x21B1)
#define CMD_READ_MEASUREMENT                       (0xEC05)
#define CMD_STOP_PERIODIC_MEASUREMENT              (0x3F86)
#define CMD_SET_TEMPERATURE_OFFSET                 (0x241D)
#define CMD_GET_TEMPERATURE_OFFSET                 (0x2318)
#define CMD_SET_SENSOR_ALTITUDE                    (0x2427)
#define CMD_GET_SENSOR_ALTITUDE                    (0x2322)
#define CMD_SET_AMBIENT_PRESSURE                   (0xE000)
#define CMD_PERFORM_FORCED_RECALIBRATION           (0x362F)
#define CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED (0x2416)
#define CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED (0x2313)
#define CMD_START_LOW_POWER_PERIODIC_MEASUREMENT   (0x21AC)
#define CMD_GET_DATA_READY_STATUS                  (0xE4B8)
#define CMD_PERSIST_SETTINGS                       (0x3615)
#define CMD_GET_SERIAL_NUMBER                      (0x3682)
#define CMD_PERFORM_SELF_TEST                      (0x3639)
#define CMD_PERFORM_FACTORY_RESET                  (0x3632)
#define CMD_REINIT                                 (0x3646)
#define CMD_MEASURE_SINGLE_SHOT                    (0x219D)
#define CMD_MEASURE_SINGLE_SHOT_RHT_ONLY           (0x2196)
#define CMD_POWER_DOWN                             (0x36E0)
#define CMD_WAKE_UP                                (0x36F6)

/* Helper macros */
#define CHECK(x) do { esp_err_t __rc = (x); if (__rc != ESP_OK) return __rc; } while (0)
#define CHECK_ARG(v) do { if (!(v)) return ESP_ERR_INVALID_ARG; } while (0)

/* CRC-8 (Sensirion): poly 0x31, init 0xFF */
static uint8_t crc8(const uint8_t *data, size_t count)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < count; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
            else crc <<= 1;
        }
    }
    return crc;
}

/* Big-endian 16-bit from two bytes */
static inline uint16_t be16(const uint8_t *p) { return (uint16_t)p[0] << 8 | (uint16_t)p[1]; }

/* Send simple 16-bit command (big-endian) */
static esp_err_t send_cmd_only(scd4x_handle_t *h, uint16_t cmd)
{
    uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_transmit(h->dev, buf, sizeof(buf), -1);
}

/* Send command + N 16-bit words (each with CRC) */
static esp_err_t send_cmd_with_words(scd4x_handle_t *h, uint16_t cmd, const uint16_t *words, size_t word_count)
{
    size_t tx_len = 2 + word_count * 3;
    uint8_t *tx = (uint8_t *)malloc(tx_len);
    if (!tx) return ESP_ERR_NO_MEM;
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFF);
    for (size_t i = 0; i < word_count; ++i) {
        uint16_t w = words[i];
        tx[2 + i*3 + 0] = (uint8_t)(w >> 8);
        tx[2 + i*3 + 1] = (uint8_t)(w & 0xFF);
        tx[2 + i*3 + 2] = crc8(&tx[2 + i*3 + 0], 2);
    }
    esp_err_t res = i2c_master_transmit(h->dev, tx, tx_len, -1);
    free(tx);
    return res;
}

/* Read in `word_count` 16-bit words (each 3 bytes: hi, lo, crc).
 * Caller must have executed the corresponding command first.
 * delay_ms can be used if sensor needs time to prepare response.
 */
static esp_err_t read_words_into(scd4x_handle_t *h, uint16_t *out_words, size_t word_count, TickType_t delay_ms)
{
    if (delay_ms) vTaskDelay(delay_ms);
    size_t rx_len = word_count * 3;
    uint8_t *rx = (uint8_t *)malloc(rx_len);
    if (!rx) return ESP_ERR_NO_MEM;
    esp_err_t err = i2c_master_receive(h->dev, rx, rx_len, -1);
    if (err != ESP_OK) { free(rx); return err; }

    for (size_t i = 0; i < word_count; ++i) {
        uint8_t *p = rx + i*3;
        uint8_t expected = crc8(p, 2);
        if (expected != p[2]) {
            ESP_LOGE(TAG, "CRC mismatch idx %u: calc 0x%02x got 0x%02x", (unsigned)i, expected, p[2]);
            free(rx);
            return ESP_ERR_INVALID_CRC;
        }
        out_words[i] = be16(p);
    }
    free(rx);
    return ESP_OK;
}

/* High-level helper: execute command that returns words (send cmd, optional delay, read words) */
static esp_err_t exec_cmd_read(scd4x_handle_t *h, uint16_t cmd, uint16_t *out_words, size_t out_word_count, TickType_t delay_ms)
{
    CHECK_ARG(h && h->dev);
    CHECK(send_cmd_only(h, cmd));
    return read_words_into(h, out_words, out_word_count, delay_ms);
}

/* High-level helper: execute command with arguments and optional response */
static esp_err_t exec_cmd_with_args_and_resp(scd4x_handle_t *h, uint16_t cmd,
                                             const uint16_t *args, size_t args_words,
                                             uint16_t *resp, size_t resp_words,
                                             TickType_t delay_ms)
{
    CHECK_ARG(h && h->dev);
    if (args_words) CHECK(send_cmd_with_words(h, cmd, args, args_words));
    else CHECK(send_cmd_only(h, cmd));
    if (resp_words) return read_words_into(h, resp, resp_words, delay_ms);
    if (delay_ms) vTaskDelay(delay_ms);
    return ESP_OK;
}

/* Public API */

esp_err_t scd4x_init(scd4x_handle_t *sensor_out, i2c_master_bus_handle_t bus_handle)
{
    CHECK_ARG(sensor_out && bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD4X_I2C_ADDRESS,
        .scl_speed_hz = SCD4X_I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &sensor_out->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Successfully initialized SCD4x");
    sensor_out->bus = bus_handle;
    return ESP_OK;
}

esp_err_t scd4x_create_bus_and_init(i2c_master_bus_handle_t *bus_out, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(bus_out);  // make sure the caller provided a valid pointer

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = scl_gpio,
        .sda_io_num = sda_gpio,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    // Create the I2C bus and store the handle at bus_out
    esp_err_t err = i2c_new_master_bus(&bus_cfg, bus_out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C bus created successfully");
    return ESP_OK;
}

esp_err_t scd4x_free(scd4x_handle_t *sensor, bool free_bus)
{
    if (!sensor) return ESP_ERR_INVALID_ARG;
    if (sensor->dev) {
        i2c_master_bus_rm_device(sensor->dev);
        sensor->dev = NULL;
    }
    if (free_bus && sensor->bus) {
        i2c_del_master_bus(sensor->bus);
        sensor->bus = NULL;
    }
    return ESP_OK;
}

/* Periodic measurement */
esp_err_t scd4x_start_periodic_measurement(scd4x_handle_t *sensor)
{
    return send_cmd_only(sensor, CMD_START_PERIODIC_MEASUREMENT);
}
esp_err_t scd4x_stop_periodic_measurement(scd4x_handle_t *sensor)
{
    return send_cmd_only(sensor, CMD_STOP_PERIODIC_MEASUREMENT);
}

/* Low power periodic measurement */
esp_err_t scd4x_start_low_power_periodic_measurement(scd4x_handle_t *sensor)
{
    return send_cmd_only(sensor, CMD_START_LOW_POWER_PERIODIC_MEASUREMENT);
}

/* Read measurement ticks (raw words) */
esp_err_t scd4x_read_measurement_ticks(scd4x_handle_t *sensor, uint16_t *co2, uint16_t *temperature, uint16_t *humidity)
{
    CHECK_ARG(sensor);
    uint16_t buf[3];
    CHECK(exec_cmd_read(sensor, CMD_READ_MEASUREMENT, buf, 3, pdMS_TO_TICKS(5)));
    if (co2) *co2 = buf[0];
    if (temperature) *temperature = buf[1];
    if (humidity) *humidity = buf[2];
    return ESP_OK;
}

/* Read measurement converted */
esp_err_t scd4x_read_measurement(scd4x_handle_t *sensor, uint16_t *co2, float *temperature, float *humidity)
{
    CHECK_ARG(sensor && (co2 || temperature || humidity));
    uint16_t t_raw = 0, h_raw = 0;
    CHECK(scd4x_read_measurement_ticks(sensor, co2, &t_raw, &h_raw));
    if (temperature) *temperature = -45.0f + 175.0f * ((float)t_raw / 65536.0f);
    if (humidity) *humidity = 100.0f * ((float)h_raw / 65536.0f);
    return ESP_OK;
}

/* Data ready */
esp_err_t scd4x_get_data_ready_status(scd4x_handle_t *sensor, bool *data_ready)
{
    CHECK_ARG(sensor && data_ready);
    uint16_t status = 0;
    CHECK(exec_cmd_read(sensor, CMD_GET_DATA_READY_STATUS, &status, 1, pdMS_TO_TICKS(1)));
    *data_ready = (status & 0x07FF) != 0;
    return ESP_OK;
}

/* Temperature offset */
esp_err_t scd4x_get_temperature_offset_ticks(scd4x_handle_t *sensor, uint16_t *t_offset)
{
    CHECK_ARG(sensor && t_offset);
    CHECK(exec_cmd_read(sensor, CMD_GET_TEMPERATURE_OFFSET, t_offset, 1, pdMS_TO_TICKS(1)));
    return ESP_OK;
}
esp_err_t scd4x_get_temperature_offset(scd4x_handle_t *sensor, float *t_offset)
{
    CHECK_ARG(sensor && t_offset);
    uint16_t raw = 0;
    CHECK(scd4x_get_temperature_offset_ticks(sensor, &raw));
    *t_offset = (float)raw * 175.0f / 65536.0f;
    return ESP_OK;
}
esp_err_t scd4x_set_temperature_offset_ticks(scd4x_handle_t *sensor, uint16_t t_offset)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_SET_TEMPERATURE_OFFSET, &t_offset, 1, NULL, 0, pdMS_TO_TICKS(1));
}
esp_err_t scd4x_set_temperature_offset(scd4x_handle_t *sensor, float t_offset)
{
    CHECK_ARG(sensor);
    uint16_t raw = (uint16_t)(t_offset * 65536.0f / 175.0f + 0.5f);
    return scd4x_set_temperature_offset_ticks(sensor, raw);
}

/* Altitude */
esp_err_t scd4x_get_sensor_altitude(scd4x_handle_t *sensor, uint16_t *altitude)
{
    CHECK_ARG(sensor && altitude);
    CHECK(exec_cmd_read(sensor, CMD_GET_SENSOR_ALTITUDE, altitude, 1, pdMS_TO_TICKS(1)));
    return ESP_OK;
}
esp_err_t scd4x_set_sensor_altitude(scd4x_handle_t *sensor, uint16_t altitude)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_SET_SENSOR_ALTITUDE, &altitude, 1, NULL, 0, pdMS_TO_TICKS(1));
}

/* Ambient pressure (hPa) */
esp_err_t scd4x_set_ambient_pressure(scd4x_handle_t *sensor, uint16_t pressure_hpa)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_SET_AMBIENT_PRESSURE, &pressure_hpa, 1, NULL, 0, pdMS_TO_TICKS(1));
}

/* Forced recalibration (FRC) */
esp_err_t scd4x_perform_forced_recalibration(scd4x_handle_t *sensor, uint16_t target_co2_concentration, uint16_t *frc_correction)
{
    CHECK_ARG(sensor && frc_correction);
    uint16_t arg = target_co2_concentration;
    CHECK(exec_cmd_with_args_and_resp(sensor, CMD_PERFORM_FORCED_RECALIBRATION, &arg, 1, frc_correction, 1, pdMS_TO_TICKS(400)));
    return ESP_OK;
}

/* Automatic self calibration (ASC) */
esp_err_t scd4x_get_automatic_self_calibration(scd4x_handle_t *sensor, bool *enabled)
{
    CHECK_ARG(sensor && enabled);
    uint16_t e = 0;
    CHECK(exec_cmd_read(sensor, CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED, &e, 1, pdMS_TO_TICKS(1)));
    *enabled = !!e;
    return ESP_OK;
}
esp_err_t scd4x_set_automatic_self_calibration(scd4x_handle_t *sensor, bool enabled)
{
    CHECK_ARG(sensor);
    uint16_t e = enabled ? 1 : 0;
    return exec_cmd_with_args_and_resp(sensor, CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED, &e, 1, NULL, 0, pdMS_TO_TICKS(1));
}

/* Persist settings */
esp_err_t scd4x_persist_settings(scd4x_handle_t *sensor)
{
    CHECK_ARG(sensor);
    return send_cmd_only(sensor, CMD_PERSIST_SETTINGS) == ESP_OK ? (vTaskDelay(pdMS_TO_TICKS(800)), ESP_OK) : ESP_FAIL;
}

/* Serial number */
esp_err_t scd4x_get_serial_number(scd4x_handle_t *sensor, uint16_t *serial0, uint16_t *serial1, uint16_t *serial2)
{
    CHECK_ARG(sensor && serial0 && serial1 && serial2);
    uint16_t buf[3];
    CHECK(exec_cmd_read(sensor, CMD_GET_SERIAL_NUMBER, buf, 3, pdMS_TO_TICKS(1)));
    *serial0 = buf[0]; *serial1 = buf[1]; *serial2 = buf[2];
    return ESP_OK;
}

/* Self test, factory reset, reinit */
esp_err_t scd4x_perform_self_test(scd4x_handle_t *sensor, bool *malfunction)
{
    CHECK_ARG(sensor && malfunction);
    uint16_t out;
    CHECK(exec_cmd_read(sensor, CMD_PERFORM_SELF_TEST, &out, 1, pdMS_TO_TICKS(10000)));
    *malfunction = (out != 0);
    return ESP_OK;
}
esp_err_t scd4x_perform_factory_reset(scd4x_handle_t *sensor)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_PERFORM_FACTORY_RESET, NULL, 0, NULL, 0, pdMS_TO_TICKS(800));
}
esp_err_t scd4x_reinit(scd4x_handle_t *sensor)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_REINIT, NULL, 0, NULL, 0, pdMS_TO_TICKS(20));
}

/* Single-shot */
esp_err_t scd4x_measure_single_shot(scd4x_handle_t *sensor)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_MEASURE_SINGLE_SHOT, NULL, 0, NULL, 0, pdMS_TO_TICKS(5000));
}
esp_err_t scd4x_measure_single_shot_rht_only(scd4x_handle_t *sensor)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_MEASURE_SINGLE_SHOT_RHT_ONLY, NULL, 0, NULL, 0, pdMS_TO_TICKS(50));
}

/* Power down / wake up */
esp_err_t scd4x_power_down(scd4x_handle_t *sensor)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_POWER_DOWN, NULL, 0, NULL, 0, pdMS_TO_TICKS(1));
}
esp_err_t scd4x_wake_up(scd4x_handle_t *sensor)
{
    CHECK_ARG(sensor);
    return exec_cmd_with_args_and_resp(sensor, CMD_WAKE_UP, NULL, 0, NULL, 0, pdMS_TO_TICKS(20));
}
