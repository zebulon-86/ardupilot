/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#ifndef LSM6DSV_DEFAULT_ROTATION
#define LSM6DSV_DEFAULT_ROTATION ROTATION_NONE
#endif

class AP_InertialSensor_LSM6DSV : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation=LSM6DSV_DEFAULT_ROTATION);

    void start() override;
    bool update() override;
    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    enum class Variant : uint8_t {
        LSM6DSV16X,
        LSM6DSV32X,
        LSM6DSK320X,
    };

    enum class AccelRoute : uint8_t {
        Primary,
        HighG,
        DualChannel,
    };

    enum class SampleSourceMode : uint8_t {
        Polling,
        FIFO,
    };

    enum class FifoTag : uint8_t {
        Empty = 0x00,
        GyroNC = 0x01,
        AccelNC = 0x02,
        Temperature = 0x03,
        Timestamp = 0x04,
        CfgChange = 0x05,
        AccelNC_T2 = 0x06,
        AccelNC_T1 = 0x07,
        Accel2xC = 0x08,
        Accel3xC = 0x09,
        GyroNC_T2 = 0x0A,
        GyroNC_T1 = 0x0B,
        Gyro2xC = 0x0C,
        Gyro3xC = 0x0D,
        RouteExt = 0x1D,
        Unsupported = 0xFF,
    };

    // VariantInfo keeps the board-selected capabilities together with the
    // stable register presets used to bring each part up today.
    struct VariantInfo {
        Variant variant;
        uint8_t whoami;
        enum DevTypes devtype;
        const char *name;
        bool has_high_g;
        bool has_dual_channel;
        uint8_t max_haodr_sel;
        uint8_t ctrl6_value;
        uint8_t ctrl8_value;
        float gyro_scale;
        float accel_scale;
        float dual_channel_accel_scale;
        float dual_channel_accel_range_g;
    };

    struct PollDebugStats {
        uint32_t callbacks;
        uint32_t publishes;
        uint32_t temp_updates;
        uint32_t ready_skips;
        uint32_t accel_not_ready;
        uint32_t gyro_not_ready;
        uint32_t dualc_both_ready;
        uint32_t dualc_gyro_only_ready;
        uint32_t dualc_accel_only_ready;
        uint32_t dualc_neither_ready;
        uint32_t read_failures;
        uint32_t register_failures;
        uint32_t fifo_words_read;
        uint32_t fifo_gyro_nc_tags;
        uint32_t fifo_accel_nc_tags;
        uint32_t fifo_temperature_tags;
        uint32_t fifo_timestamp_tags;
        uint32_t fifo_cfg_change_tags;
        uint32_t fifo_accel_ext_tags;
        uint32_t fifo_gyro_ext_tags;
        uint32_t fifo_other_tags;
        uint32_t fifo_recent_raw_tags;
        uint32_t gap_over_1ms;
        uint32_t gap_over_5ms;
        uint32_t gap_over_10ms;
        uint32_t max_gap_us;
        uint32_t last_poll_us;
        uint32_t last_report_ms;
    };

    struct SampleFrame {
        Vector3f gyro;
        Vector3f accel;
        int16_t raw_temp;
        bool has_raw_temp;
    };

    struct PollingFrame {
        SampleFrame sample;
        uint8_t gyro_status;
        uint8_t accel_status;
    };

    struct SourceFrame {
        SampleFrame sample;
        uint8_t gyro_status;
        uint8_t accel_status;
    };

    struct FifoFrame {
        SourceFrame source;
        FifoTag tag;
        uint16_t unread_words;
        uint8_t tag_count;
    };

    AP_InertialSensor_LSM6DSV(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    bool init();
    bool hardware_init();
    const VariantInfo *detect_variant();
    bool reset_device();
    // Select and activate the accel data source for this backend instance.
    bool prepare_accel_mode(uint8_t accel_instance_hint);
    bool configure_gyro();
    bool configure_accel();
    bool configure_primary_fifo();
    bool configure_high_g_accel(uint8_t mode);
    bool configure_dual_channel_accel();
    bool wait_for_data_ready();

    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    bool write_register(uint8_t reg, uint8_t value, bool checked=false);
    bool fetch_primary_sample(SampleFrame &sample);
    bool fetch_dual_channel_sample(SampleFrame &sample);
    bool fetch_high_g_sample(SampleFrame &sample);
    // Read one gyro+accel sample using the currently selected accel route.
    bool read_sample(SampleFrame &sample);
    bool read_status_registers(uint8_t &gyro_status, uint8_t &accel_status, uint32_t now_us);
    bool sample_ready_for_route(uint8_t gyro_status, uint8_t accel_status, uint32_t now_us);
    bool fetch_current_sample(SampleFrame &sample, uint32_t now_us);
    bool fetch_polling_frame(PollingFrame &frame, uint32_t now_us);
    bool fetch_source_frame(SourceFrame &frame, uint32_t now_us);
    bool read_fifo_status(FifoFrame &frame, uint32_t now_us);
    bool read_fifo_words_block(uint16_t n_words, uint32_t now_us);
    bool consume_fifo_word(FifoFrame &frame, SampleFrame &sample, const uint8_t *raw_word);
    uint16_t drain_fifo(uint32_t now_us);
    void publish_gyro_sample(SampleFrame &sample);
    void publish_accel_sample(SampleFrame &sample);
    void publish_current_sample(SampleFrame &sample, uint8_t gyro_status);
    void check_register_monitor();
    void update_temperature(uint8_t status, const int16_t *raw_temp=nullptr);
    uint8_t odr_code_for_rate(uint16_t rate_hz) const;
    uint16_t calculate_backend_rate(uint16_t base_rate_hz) const;
    uint8_t high_g_mode_for_instance(uint8_t accel_instance_hint) const;
    bool dual_channel_mode_for_instance(uint8_t accel_instance_hint) const;
    char accel_route_code() const;
    const char *temperature_mode_name() const;
    const char *sample_mode_name() const;
    SampleSourceMode active_sample_source() const;
    bool fifo_supported_for_current_route() const;
    static bool fifo_tag_supported_for_primary(FifoTag tag);
    static FifoTag decode_fifo_tag(uint8_t raw_tag);
    static uint8_t decode_fifo_tag_count(uint8_t raw_tag);
    void maybe_log_poll_debug(uint32_t now_us);
    void update_dual_channel_ready_stats(bool gyro_ready, bool accel_ready);

    // Periodic callback entry point: readiness checks, sample acquisition,
    // publishing, and runtime stats all flow through here.
    void poll_data();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;
    const VariantInfo *_variant_info;
    enum Rotation _rotation;
    float _accel_scale;
    float _gyro_scale;
    float _accel_range_g;
    uint8_t _whoami;
    uint8_t _accel_output_reg;
    uint8_t _accel_status_reg;
    uint8_t _accel_ready_mask;
    uint8_t _temperature_counter;
    uint16_t _backend_rate_hz;
    uint32_t _backend_period_us;
    bool _fast_sampling;
    AccelRoute _accel_route;
    PollDebugStats _poll_debug{};
    uint8_t *_fifo_buffer;

    static const VariantInfo variant_info[];
};
