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
/*
  driver for ST LSM6DSV-family IMUs including LSM6DSV16X, LSM6DSV32X
  and LSM6DSK320X (high-G variant)

  Uses HAODR mode-1 for high-accuracy ODR (1000-8000 Hz) and
  continuous FIFO for burst reads.  Digital filters auto-adapt to ODR
  so no per-rate AAF register reconfiguration is needed.

  fast sampling is controlled via INS_FAST_SAMPLE / INS_GYRO_RATE
  with base rate 1000 Hz.
 */

#include "AP_InertialSensor_LSM6DSV.h"

#include <stdio.h>
#include <utility>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

namespace {

#ifndef LSM6DSV_TIMING_DEBUG_ENABLED
#define LSM6DSV_TIMING_DEBUG_ENABLED 1
#endif

#ifndef LSM6DSV_EXPERIMENTAL_PRIMARY_FIFO
#define LSM6DSV_EXPERIMENTAL_PRIMARY_FIFO 1
#endif

// registers
constexpr uint8_t LSM6DSV_REG_FUNC_CFG_ACCESS = 0x01;
constexpr uint8_t LSM6DSV_REG_FIFO_CTRL1 = 0x07;
constexpr uint8_t LSM6DSV_REG_FIFO_CTRL2 = 0x08;
constexpr uint8_t LSM6DSV_REG_FIFO_CTRL3 = 0x09;
constexpr uint8_t LSM6DSV_REG_FIFO_CTRL4 = 0x0A;
constexpr uint8_t LSM6DSV_REG_WHO_AM_I   = 0x0F;
constexpr uint8_t LSM6DSV_REG_CTRL1      = 0x10;
constexpr uint8_t LSM6DSV_REG_CTRL2      = 0x11;
constexpr uint8_t LSM6DSV_REG_CTRL3      = 0x12;
constexpr uint8_t LSM6DSV_REG_CTRL6      = 0x15;
constexpr uint8_t LSM6DSV_REG_CTRL8      = 0x17;
constexpr uint8_t LSM6DSV_REG_FIFO_STATUS1 = 0x1B;
constexpr uint8_t LSM6DSV_REG_FIFO_STATUS2 = 0x1C;
constexpr uint8_t LSM6DSV_REG_STATUS     = 0x1E;
constexpr uint8_t LSM6DSV_REG_OUT_TEMP_L = 0x20;
constexpr uint8_t LSM6DSV_REG_OUTX_L_G   = 0x22;
constexpr uint8_t LSM6DSV_REG_OUTX_L_A   = 0x28;
constexpr uint8_t LSM6DSV_REG_OUTX_L_A_HG = 0x34;
constexpr uint8_t LSM6DSV_REG_UI_STATUS_OIS = 0x44;
constexpr uint8_t LSM6DSV_REG_CTRL2_XL_HG = 0x4D;
constexpr uint8_t LSM6DSV_REG_CTRL1_XL_HG = 0x4E;
constexpr uint8_t LSM6DSV_REG_HAODR_CFG  = 0x62;
constexpr uint8_t LSM6DSV_REG_UI_CTRL1_OIS = 0x70;
constexpr uint8_t LSM6DSV_REG_FIFO_DATA_OUT_TAG = 0x78;
constexpr uint8_t LSM6DSV_REG_FIFO_DATA_OUT_X_L = 0x79;

constexpr uint8_t LSM6DSV_ID_LSM6DSV32X  = 0x70;
constexpr uint8_t LSM6DSV_ID_LSM6DSK320X = 0x75;

constexpr uint8_t LSM6DSV_CTRL_MODE_HAODR = 0x10;
constexpr uint8_t LSM6DSV_HAODR_CFG_MODE1 = 0x01;
// HAODR mode-1 ODR codes 
constexpr uint8_t LSM6DSV_MODE1_ODR_125HZ  = 0x06;
constexpr uint8_t LSM6DSV_MODE1_ODR_250HZ  = 0x07;
constexpr uint8_t LSM6DSV_MODE1_ODR_500HZ  = 0x08;
constexpr uint8_t LSM6DSV_MODE1_ODR_1000HZ = 0x09;
constexpr uint8_t LSM6DSV_MODE1_ODR_2000HZ = 0x0A;
constexpr uint8_t LSM6DSV_MODE1_ODR_4000HZ = 0x0B;
constexpr uint8_t LSM6DSV_MODE1_ODR_8000HZ = 0x0C;
constexpr uint8_t LSM6DSV_CTRL3_BDU       = 1U << 6;
constexpr uint8_t LSM6DSV_CTRL3_IF_INC    = 1U << 2;
constexpr uint8_t LSM6DSV_CTRL3_SW_RESET  = 1U << 0;
constexpr uint8_t LSM6DSV_STATUS_XLDA     = 1U << 0;
constexpr uint8_t LSM6DSV_STATUS_GDA      = 1U << 1;
constexpr uint8_t LSM6DSV_STATUS_TDA      = 1U << 2;
constexpr uint8_t LSM6DSV_STATUS_XLHGDA   = 1U << 4;
constexpr uint8_t LSM6DSV_STATUS_XLDA_OIS = 1U << 0;
constexpr uint8_t LSM6DSV_FIFO_STATUS2_DIFF_FIFO_8 = 1U << 0;
constexpr uint8_t LSM6DSV_FIFO_CTRL3_BDR_DISABLED = 0x00;
constexpr uint8_t LSM6DSV_FIFO_CTRL4_MODE_BYPASS = 0x00;
constexpr uint8_t LSM6DSV_FIFO_CTRL4_MODE_CONTINUOUS = 0x06;
constexpr uint8_t LSM6DSV_SPI_READ_FLAG   = 0x80;
constexpr uint8_t LSM6DSV_FUNC_CFG_OIS_CTRL_FROM_UI = 1U << 0;
constexpr uint8_t LSM6DSV_CTRL8_XL_DUALC_EN = 1U << 3;
constexpr uint8_t LSM6DSV_UI_CTRL1_OIS_XL_EN = 1U << 2;
constexpr uint8_t LSM6DSV_CTRL1_XL_HG_REGOUT_EN = 1U << 7;
constexpr uint8_t LSM6DSV_CTRL1_XL_HG_ODR_7680HZ = 0x07U << 3;
constexpr uint8_t LSM6DSV_CTRL6_PRESET_GYRO_2000DPS = 0x04;
constexpr uint8_t LSM6DSV_CTRL6_PRESET_GYRO_4000DPS = 0x0C;
constexpr uint8_t LSM6DSV_CTRL6_PRESET_GYRO_2000DPS_HIGH_G = 0x05;
constexpr uint8_t LSM6DSV_CTRL6_PRESET_GYRO_4000DPS_HIGH_G = 0x0D;
constexpr uint8_t LSM6DSV_CTRL8_PRESET_ACCEL_16G = 0x03;
constexpr uint8_t LSM6DSV_CTRL8_PRESET_ACCEL_32G = 0x06;

constexpr uint16_t LSM6DSV_DEFAULT_BACKEND_RATE_HZ = 1000;
constexpr uint8_t LSM6DSV_INIT_MAX_TRIES = 5;
constexpr uint8_t LSM6DSV_RESET_TIMEOUT_MS = 100;
constexpr uint8_t LSM6DSV_DATA_READY_TIMEOUT_MS = 20;
constexpr uint8_t LSM6DSV_POWERUP_DELAY_MS = 5;
constexpr uint32_t LSM6DSV_TIMING_DEBUG_REPORT_MS = 2000;
constexpr uint8_t LSM6DSV_TEMPERATURE_UPDATE_INTERVAL = 100;
constexpr uint16_t LSM6DSV_PRIMARY_FIFO_WATERMARK_WORDS = 2;
constexpr uint16_t LSM6DSV_FIFO_MAX_DRAIN_WORDS = 32;
constexpr uint16_t LSM6DSV_FIFO_BURST_WORDS = 16;
constexpr float LSM6DSV_TEMPERATURE_ZERO_C = 25.0f;
constexpr float LSM6DSV_TEMPERATURE_SENSITIVITY = 256.0f;
constexpr float LSM6DSV_ACCEL_SCALE_16G = GRAVITY_MSS * 16.0f / 32768.0f;
constexpr float LSM6DSV_ACCEL_SCALE_32G = GRAVITY_MSS * 32.0f / 32768.0f;
// LSM6DSV gyro sensitivity per ST datasheet (mdps/LSB), converted to rad/s
constexpr float LSM6DSV_GYRO_SCALE_125DPS = radians(4.375f / 1000.0f);
constexpr float LSM6DSV_GYRO_SCALE_250DPS = radians(8.75f / 1000.0f);
constexpr float LSM6DSV_GYRO_SCALE_500DPS = radians(17.50f / 1000.0f);
constexpr float LSM6DSV_GYRO_SCALE_1000DPS = radians(35.0f / 1000.0f);
constexpr float LSM6DSV_GYRO_SCALE_2000DPS = radians(70.0f / 1000.0f);
constexpr float LSM6DSV_GYRO_SCALE_4000DPS = radians(140.0f / 1000.0f);
constexpr float LSM6DSV_HG_ACCEL_SCALE_32G  = GRAVITY_MSS * 0.000976f;
constexpr float LSM6DSV_HG_ACCEL_SCALE_64G  = GRAVITY_MSS * 0.001952f;
constexpr float LSM6DSV_HG_ACCEL_SCALE_128G = GRAVITY_MSS * 0.003904f;
constexpr float LSM6DSV_HG_ACCEL_SCALE_256G = GRAVITY_MSS * 0.007808f;
constexpr float LSM6DSV_HG_ACCEL_SCALE_320G = GRAVITY_MSS * 0.010417f;

struct PACKED RawSample {
    le16_t gyro[3];
    le16_t accel[3];
};

struct PACKED RawSampleWithTemp {
    le16_t temp;
    le16_t gyro[3];
    le16_t accel[3];
};

struct PACKED RawAxes {
    le16_t axis[3];
};

struct PACKED RawFifoWord {
    uint8_t tag;
    le16_t axis[3];
};

static_assert(sizeof(RawSample) == 12, "RawSample must be 12 bytes");
static_assert(sizeof(RawSampleWithTemp) == 14, "RawSampleWithTemp must be 14 bytes");
static_assert(sizeof(RawAxes) == 6, "RawAxes must be 6 bytes");
static_assert(sizeof(RawFifoWord) == 7, "RawFifoWord must be 7 bytes");
constexpr uint16_t LSM6DSV_FIFO_BURST_BUFFER_SIZE = LSM6DSV_FIFO_BURST_WORDS * sizeof(RawFifoWord) + 1;

#ifndef LSM6DSV_HG_MODE_IMU0
#define LSM6DSV_HG_MODE_IMU0 0
#endif

#ifndef LSM6DSV_HG_MODE_IMU1
#define LSM6DSV_HG_MODE_IMU1 0
#endif

#ifndef LSM6DSV_HG_MODE_IMU2
#define LSM6DSV_HG_MODE_IMU2 0
#endif

#ifndef LSM6DSV_DUALC_MODE_IMU0
#define LSM6DSV_DUALC_MODE_IMU0 0
#endif

#ifndef LSM6DSV_DUALC_MODE_IMU1
#define LSM6DSV_DUALC_MODE_IMU1 0
#endif

#ifndef LSM6DSV_DUALC_MODE_IMU2
#define LSM6DSV_DUALC_MODE_IMU2 0
#endif

float lsm6dsv_high_g_scale_for_mode(const uint8_t mode)
{
    switch (mode) {
    case 1:
        return LSM6DSV_HG_ACCEL_SCALE_32G;
    case 2:
        return LSM6DSV_HG_ACCEL_SCALE_64G;
    case 3:
        return LSM6DSV_HG_ACCEL_SCALE_128G;
    case 4:
        return LSM6DSV_HG_ACCEL_SCALE_256G;
    case 5:
        return LSM6DSV_HG_ACCEL_SCALE_320G;
    default:
        return 0.0f;
    }
}

float lsm6dsv_high_g_range_for_mode(const uint8_t mode)
{
    switch (mode) {
    case 1:
        return 32.0f;
    case 2:
        return 64.0f;
    case 3:
        return 128.0f;
    case 4:
        return 256.0f;
    case 5:
        return 320.0f;
    default:
        return 0.0f;
    }
}

uint8_t lsm6dsv_high_g_ctrl1_for_mode(const uint8_t mode)
{
    if (mode < 1 || mode > 5) {
        return 0;
    }
    return LSM6DSV_CTRL1_XL_HG_REGOUT_EN | LSM6DSV_CTRL1_XL_HG_ODR_7680HZ | (mode - 1);
}

}

// Keep the current register presets close to the capability table so the
// route and variant decisions remain easy to audit together.
const AP_InertialSensor_LSM6DSV::VariantInfo AP_InertialSensor_LSM6DSV::variant_info[] = {
    {
        Variant::LSM6DSV16X,
        LSM6DSV_ID_LSM6DSV32X,
        DEVTYPE_INS_LSM6DSV,
        "LSM6DSV16X",
        false,
        true,
        2,
        LSM6DSV_CTRL6_PRESET_GYRO_2000DPS,
        LSM6DSV_CTRL8_PRESET_ACCEL_16G,
        LSM6DSV_GYRO_SCALE_2000DPS,
        LSM6DSV_ACCEL_SCALE_16G,
        LSM6DSV_ACCEL_SCALE_16G,
        16.0f,
    },
    {
        Variant::LSM6DSV32X,
        LSM6DSV_ID_LSM6DSV32X,
        DEVTYPE_INS_LSM6DSV,
        "LSM6DSV32X",
        false,
        true,
        2,
        LSM6DSV_CTRL6_PRESET_GYRO_2000DPS,
        LSM6DSV_CTRL8_PRESET_ACCEL_32G,
        LSM6DSV_GYRO_SCALE_2000DPS,
        LSM6DSV_ACCEL_SCALE_16G,
        LSM6DSV_ACCEL_SCALE_32G,
        32.0f,
    },
    {
        Variant::LSM6DSK320X,
        LSM6DSV_ID_LSM6DSK320X,
        DEVTYPE_INS_LSM6DSV,
        "LSM6DSK320X",
        true,
        false,
        3,
        LSM6DSV_CTRL6_PRESET_GYRO_2000DPS_HIGH_G,
        LSM6DSV_CTRL8_PRESET_ACCEL_16G,
        LSM6DSV_GYRO_SCALE_2000DPS,
        LSM6DSV_ACCEL_SCALE_16G,
        0.0f,
        0.0f,
    },
};

AP_InertialSensor_LSM6DSV::AP_InertialSensor_LSM6DSV(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _variant_info(nullptr)
    , _rotation(rotation)
    , _accel_scale(LSM6DSV_ACCEL_SCALE_16G)
    , _gyro_scale(LSM6DSV_GYRO_SCALE_2000DPS)
    , _accel_range_g(16.0f)
    , _whoami(0)
    , _accel_output_reg(LSM6DSV_REG_OUTX_L_A)
    , _accel_status_reg(LSM6DSV_REG_STATUS)
    , _accel_ready_mask(LSM6DSV_STATUS_XLDA)
    , _temperature_counter(0)
    , _accel_route(AccelRoute::Primary)
    , _fifo_buffer(nullptr)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSV::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto *sensor = NEW_NOTHROW AP_InertialSensor_LSM6DSV(imu, std::move(dev), rotation);
    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_LSM6DSV::start()
{
    uint8_t accel_instance_hint;
    if (!_imu.get_accel_instance(accel_instance_hint) ||
        !_imu.get_gyro_instance(gyro_instance) ||
        !prepare_accel_mode(accel_instance_hint) ||
        !_imu.register_accel(accel_instance, LSM6DSV_DEFAULT_BACKEND_RATE_HZ, _dev->get_bus_id_devtype(_variant_info->devtype)) ||
        !_imu.register_gyro(gyro_instance, LSM6DSV_DEFAULT_BACKEND_RATE_HZ, _dev->get_bus_id_devtype(_variant_info->devtype))) {
        return;
    }

    // determine fast sampling rate (SPI only)
    _backend_rate_hz = LSM6DSV_DEFAULT_BACKEND_RATE_HZ;
    if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate() > 1) {
        _fast_sampling = (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI);
    }
    if (_fast_sampling) {
        _backend_rate_hz = calculate_backend_rate(LSM6DSV_DEFAULT_BACKEND_RATE_HZ);
    }
    _backend_period_us = 1000000UL / _backend_rate_hz;

    // re-configure ODR registers for the target sampling rate
    WITH_SEMAPHORE(_dev->get_semaphore());
    const uint8_t odr = odr_code_for_rate(_backend_rate_hz);
    write_register(LSM6DSV_REG_CTRL1, LSM6DSV_CTRL_MODE_HAODR | odr, true);
    write_register(LSM6DSV_REG_CTRL2, LSM6DSV_CTRL_MODE_HAODR | odr, true);
    if (active_sample_source() == SampleSourceMode::FIFO) {
        configure_primary_fifo();
    }

    if (_accel_route != AccelRoute::Primary && !is_zero(_accel_range_g)) {
        _set_raw_sample_accel_multiplier(accel_instance,
                                         uint16_t(INT16_MAX / (_accel_range_g * GRAVITY_MSS)));
    }

    set_gyro_orientation(gyro_instance, _rotation);
    set_accel_orientation(accel_instance, _rotation);

    if (active_sample_source() == SampleSourceMode::FIFO) {
        _fifo_buffer = static_cast<uint8_t *>(hal.util->malloc_type(LSM6DSV_FIFO_BURST_BUFFER_SIZE,
                                                                     AP_HAL::Util::MEM_DMA_SAFE));
        if (_fifo_buffer == nullptr) {
            return;
        }
    }

    periodic_handle = _dev->register_periodic_callback(_backend_period_us,
                                                       FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSV::poll_data, void));
}

bool AP_InertialSensor_LSM6DSV::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_LSM6DSV::get_output_banner(char* banner, uint8_t banner_len)
{
    if (_variant_info == nullptr) {
        return false;
    }

    if (_accel_route == AccelRoute::HighG) {
        snprintf(banner, banner_len, "IMU%u: %s high-g %ug %.1fkHz temp %s sample %s",
                 gyro_instance,
                 _variant_info->name,
                 static_cast<unsigned>(_accel_range_g),
                 _backend_rate_hz * 0.001f,
                 temperature_mode_name(),
                 sample_mode_name());
        return true;
    }

    if (_accel_route == AccelRoute::DualChannel) {
        snprintf(banner, banner_len, "IMU%u: %s dual-channel %.1fkHz temp %s sample %s",
                 gyro_instance,
                 _variant_info->name,
                 _backend_rate_hz * 0.001f,
                 temperature_mode_name(),
                 sample_mode_name());
        return true;
    }

    snprintf(banner, banner_len, "IMU%u: %s primary %.1fkHz temp %s sample %s",
             gyro_instance,
             _variant_info->name,
             _backend_rate_hz * 0.001f,
             temperature_mode_name(),
             sample_mode_name());
    return true;
}

bool AP_InertialSensor_LSM6DSV::init()
{
    _dev->set_read_flag(LSM6DSV_SPI_READ_FLAG);
    return hardware_init();
}

bool AP_InertialSensor_LSM6DSV::hardware_init()
{
    hal.scheduler->delay(LSM6DSV_POWERUP_DELAY_MS);

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    if (!_dev->setup_checked_registers(13, 20)) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < LSM6DSV_INIT_MAX_TRIES; attempt++) {
        _variant_info = detect_variant();
        if (_variant_info == nullptr) {
            continue;
        }
        _gyro_scale = _variant_info->gyro_scale;
        _accel_scale = _variant_info->accel_scale;

        if (!reset_device()) {
            continue;
        }

        if (!configure_gyro()) {
            continue;
        }

        if (!configure_accel()) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_HAODR_CFG, LSM6DSV_HAODR_CFG_MODE1, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_CTRL1, LSM6DSV_CTRL_MODE_HAODR | LSM6DSV_MODE1_ODR_1000HZ, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_CTRL2, LSM6DSV_CTRL_MODE_HAODR | LSM6DSV_MODE1_ODR_1000HZ, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_CTRL3, LSM6DSV_CTRL3_BDU | LSM6DSV_CTRL3_IF_INC, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_FIFO_CTRL4, 0x00, true)) {
            continue;
        }

        if (!wait_for_data_ready()) {
            continue;
        }

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        return true;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    return false;
}

const AP_InertialSensor_LSM6DSV::VariantInfo *AP_InertialSensor_LSM6DSV::detect_variant()
{
    if (!read_registers(LSM6DSV_REG_WHO_AM_I, &_whoami, 1)) {
        return nullptr;
    }

    if (_whoami == LSM6DSV_ID_LSM6DSV32X) {
        switch (AP_BoardConfig::get_board_type()) {
        case AP_BoardConfig::FMUV6_BOARD_HOLYBRO_6C_LSM6DSV16X:
            for (const auto &info : variant_info) {
                if (info.variant == Variant::LSM6DSV16X) {
                    return &info;
                }
            }
            break;
        default:
            break;
        }
    }

    for (const auto &info : variant_info) {
        if (info.whoami == _whoami && info.variant != Variant::LSM6DSV16X) {
            return &info;
        }
    }

    return nullptr;
}

bool AP_InertialSensor_LSM6DSV::reset_device()
{
    if (!write_register(LSM6DSV_REG_CTRL3, LSM6DSV_CTRL3_SW_RESET)) {
        return false;
    }

    for (uint8_t i = 0; i < LSM6DSV_RESET_TIMEOUT_MS; i++) {
        uint8_t ctrl3 = 0;
        hal.scheduler->delay(1);
        if (!read_registers(LSM6DSV_REG_CTRL3, &ctrl3, 1)) {
            continue;
        }
        if ((ctrl3 & LSM6DSV_CTRL3_SW_RESET) == 0) {
            return true;
        }
    }

    return false;
}

bool AP_InertialSensor_LSM6DSV::configure_gyro()
{
    return write_register(LSM6DSV_REG_CTRL6, _variant_info->ctrl6_value, true);
}

bool AP_InertialSensor_LSM6DSV::configure_accel()
{
    return write_register(LSM6DSV_REG_CTRL8, _variant_info->ctrl8_value, true);
}

bool AP_InertialSensor_LSM6DSV::configure_primary_fifo()
{
#if !LSM6DSV_EXPERIMENTAL_PRIMARY_FIFO
    return true;
#else
    const uint8_t odr = odr_code_for_rate(_backend_rate_hz);
    const uint8_t fifo_ctrl3 = uint8_t((odr << 4) | odr);

    return write_register(LSM6DSV_REG_FIFO_CTRL1,
                          uint8_t(LSM6DSV_PRIMARY_FIFO_WATERMARK_WORDS & 0xFFU),
                          true) &&
           write_register(LSM6DSV_REG_FIFO_CTRL2, 0x00, true) &&
           write_register(LSM6DSV_REG_FIFO_CTRL3, fifo_ctrl3, true) &&
           write_register(LSM6DSV_REG_FIFO_CTRL4, LSM6DSV_FIFO_CTRL4_MODE_CONTINUOUS, true);
#endif
}

bool AP_InertialSensor_LSM6DSV::configure_high_g_accel(const uint8_t mode)
{
    const uint8_t ctrl1 = lsm6dsv_high_g_ctrl1_for_mode(mode);
    if (ctrl1 == 0) {
        return false;
    }

    return write_register(LSM6DSV_REG_CTRL1_XL_HG, ctrl1, true);
}

bool AP_InertialSensor_LSM6DSV::configure_dual_channel_accel()
{
    uint8_t func_cfg = 0;
    if (!read_registers(LSM6DSV_REG_FUNC_CFG_ACCESS, &func_cfg, 1)) {
        return false;
    }
    if (!write_register(LSM6DSV_REG_FUNC_CFG_ACCESS,
                        func_cfg | LSM6DSV_FUNC_CFG_OIS_CTRL_FROM_UI,
                        true)) {
        return false;
    }

    uint8_t ui_ctrl1_ois = 0;
    if (!read_registers(LSM6DSV_REG_UI_CTRL1_OIS, &ui_ctrl1_ois, 1)) {
        return false;
    }
    if (!write_register(LSM6DSV_REG_UI_CTRL1_OIS,
                        ui_ctrl1_ois | LSM6DSV_UI_CTRL1_OIS_XL_EN,
                        true)) {
        return false;
    }

    return write_register(LSM6DSV_REG_CTRL8,
                          _variant_info->ctrl8_value | LSM6DSV_CTRL8_XL_DUALC_EN,
                          true);
}

uint8_t AP_InertialSensor_LSM6DSV::odr_code_for_rate(uint16_t rate_hz) const
{
    switch (rate_hz) {
    case 8000: return LSM6DSV_MODE1_ODR_8000HZ;
    case 4000: return LSM6DSV_MODE1_ODR_4000HZ;
    case 2000: return LSM6DSV_MODE1_ODR_2000HZ;
    case 1000:
    default:   return LSM6DSV_MODE1_ODR_1000HZ;
    }
}

// calculate the backend sample rate accounting for fast sampling
// multiplier and loop rate constraints
uint16_t AP_InertialSensor_LSM6DSV::calculate_backend_rate(uint16_t base_rate_hz) const
{
    // constrain the gyro rate to be at least the loop rate
    uint8_t min_mult = 1;
    if (get_loop_rate_hz() > base_rate_hz) {
        min_mult = 2;
    }
    if (get_loop_rate_hz() > base_rate_hz * 2) {
        min_mult = 4;
    }
    const uint8_t mult = constrain_int16(get_fast_sampling_rate(), min_mult, 8);
    return constrain_int16(base_rate_hz * mult, base_rate_hz, 8000);
}

uint8_t AP_InertialSensor_LSM6DSV::high_g_mode_for_instance(const uint8_t accel_instance_hint) const
{
    switch (accel_instance_hint) {
    case 0:
        return LSM6DSV_HG_MODE_IMU0;
    case 1:
        return LSM6DSV_HG_MODE_IMU1;
    case 2:
        return LSM6DSV_HG_MODE_IMU2;
    default:
        return 0;
    }
}

bool AP_InertialSensor_LSM6DSV::dual_channel_mode_for_instance(const uint8_t accel_instance_hint) const
{
    switch (accel_instance_hint) {
    case 0:
        return LSM6DSV_DUALC_MODE_IMU0 != 0;
    case 1:
        return LSM6DSV_DUALC_MODE_IMU1 != 0;
    case 2:
        return LSM6DSV_DUALC_MODE_IMU2 != 0;
    default:
        return false;
    }
}

char AP_InertialSensor_LSM6DSV::accel_route_code() const
{
    switch (_accel_route) {
    case AccelRoute::HighG:
        return 'H';
    case AccelRoute::DualChannel:
        return 'D';
    case AccelRoute::Primary:
    default:
        return 'P';
    }
}

const char *AP_InertialSensor_LSM6DSV::temperature_mode_name() const
{
    if (active_sample_source() == SampleSourceMode::FIFO) {
        return "polled";
    }
    return _accel_route == AccelRoute::Primary ? "burst" : "polled";
}

const char *AP_InertialSensor_LSM6DSV::sample_mode_name() const
{
    if (active_sample_source() == SampleSourceMode::FIFO) {
        return "fifo-exp";
    }
    return _accel_route == AccelRoute::Primary ? "14B" : "split";
}

AP_InertialSensor_LSM6DSV::SampleSourceMode AP_InertialSensor_LSM6DSV::active_sample_source() const
{
    // Experimental FIFO bring-up is intentionally limited to the primary
    // accel route so dual-channel and high-g users keep the known-good path.
#if LSM6DSV_EXPERIMENTAL_PRIMARY_FIFO
    if (fifo_supported_for_current_route()) {
        return SampleSourceMode::FIFO;
    }
#endif
    return SampleSourceMode::Polling;
}

bool AP_InertialSensor_LSM6DSV::fifo_supported_for_current_route() const
{
    return _accel_route == AccelRoute::Primary;
}

bool AP_InertialSensor_LSM6DSV::fifo_tag_supported_for_primary(const FifoTag tag)
{
    return tag == FifoTag::GyroNC || tag == FifoTag::AccelNC;
}

AP_InertialSensor_LSM6DSV::FifoTag AP_InertialSensor_LSM6DSV::decode_fifo_tag(const uint8_t raw_tag)
{
    switch ((raw_tag >> 3) & 0x1FU) {
    case 0x00:
        return FifoTag::Empty;
    case 0x01:
        return FifoTag::GyroNC;
    case 0x02:
        return FifoTag::AccelNC;
    case 0x03:
        return FifoTag::Temperature;
    case 0x04:
        return FifoTag::Timestamp;
    case 0x05:
        return FifoTag::CfgChange;
    case 0x06:
        return FifoTag::AccelNC_T2;
    case 0x07:
        return FifoTag::AccelNC_T1;
    case 0x08:
        return FifoTag::Accel2xC;
    case 0x09:
        return FifoTag::Accel3xC;
    case 0x0A:
        return FifoTag::GyroNC_T2;
    case 0x0B:
        return FifoTag::GyroNC_T1;
    case 0x0C:
        return FifoTag::Gyro2xC;
    case 0x0D:
        return FifoTag::Gyro3xC;
    case 0x1D:
        return FifoTag::RouteExt;
    default:
        return FifoTag::Unsupported;
    }
}

uint8_t AP_InertialSensor_LSM6DSV::decode_fifo_tag_count(const uint8_t raw_tag)
{
    return (raw_tag >> 1) & 0x03U;
}

void AP_InertialSensor_LSM6DSV::maybe_log_poll_debug(const uint32_t now_us)
{
#if LSM6DSV_TIMING_DEBUG_ENABLED
    const uint32_t now_ms = now_us / 1000U;
    if (_poll_debug.last_report_ms == 0) {
        _poll_debug.last_report_ms = now_ms;
        return;
    }
    if ((now_ms - _poll_debug.last_report_ms) < LSM6DSV_TIMING_DEBUG_REPORT_MS) {
        return;
    }

    char route_name[2] { accel_route_code(), '\0' };
    AP::logger().WriteCritical(
        "L6T",
        "TimeUS,Ins,Route,Cb,Ok,Temp,Rd,Ar,Gr,Rf,Reg",
        "QBNIIIIIIII",
        AP_HAL::micros64(),
        accel_instance,
        route_name,
        _poll_debug.callbacks,
        _poll_debug.publishes,
        _poll_debug.temp_updates,
        _poll_debug.ready_skips,
        _poll_debug.accel_not_ready,
        _poll_debug.gyro_not_ready,
        _poll_debug.read_failures,
        _poll_debug.register_failures);
    if (_accel_route == AccelRoute::DualChannel) {
        AP::logger().WriteCritical(
            "L6D",
            "TimeUS,Ins,Route,Both,GyroOnly,AccelOnly,Neither",
            "QBNIIII",
            AP_HAL::micros64(),
            accel_instance,
            route_name,
            _poll_debug.dualc_both_ready,
            _poll_debug.dualc_gyro_only_ready,
            _poll_debug.dualc_accel_only_ready,
            _poll_debug.dualc_neither_ready);
    }
    AP::logger().WriteCritical(
        "L6G",
        "TimeUS,Ins,Route,MaxGap,Gap1ms,Gap5ms,Gap10ms",
        "QBNIIII",
        AP_HAL::micros64(),
        accel_instance,
        route_name,
        _poll_debug.max_gap_us,
        _poll_debug.gap_over_1ms,
        _poll_debug.gap_over_5ms,
        _poll_debug.gap_over_10ms);

    if (active_sample_source() == SampleSourceMode::FIFO &&
        _accel_route == AccelRoute::Primary) {
        AP::logger().WriteCritical(
            "L6F",
            "TimeUS,Ins,Route,Words,GTag,ATag,Cfg,Temp,Ts,AExt,GExt,Other,Recent",
            "QBNIIIIIIIIIII",
            AP_HAL::micros64(),
            accel_instance,
            route_name,
            _poll_debug.fifo_words_read,
            _poll_debug.fifo_gyro_nc_tags,
            _poll_debug.fifo_accel_nc_tags,
            _poll_debug.fifo_cfg_change_tags,
            _poll_debug.fifo_temperature_tags,
            _poll_debug.fifo_timestamp_tags,
            _poll_debug.fifo_accel_ext_tags,
            _poll_debug.fifo_gyro_ext_tags,
            _poll_debug.fifo_other_tags,
            _poll_debug.fifo_recent_raw_tags);
    }

    _poll_debug.callbacks = 0;
    _poll_debug.publishes = 0;
    _poll_debug.temp_updates = 0;
    _poll_debug.ready_skips = 0;
    _poll_debug.accel_not_ready = 0;
    _poll_debug.gyro_not_ready = 0;
    _poll_debug.dualc_both_ready = 0;
    _poll_debug.dualc_gyro_only_ready = 0;
    _poll_debug.dualc_accel_only_ready = 0;
    _poll_debug.dualc_neither_ready = 0;
    _poll_debug.read_failures = 0;
    _poll_debug.register_failures = 0;
    _poll_debug.fifo_words_read = 0;
    _poll_debug.fifo_gyro_nc_tags = 0;
    _poll_debug.fifo_accel_nc_tags = 0;
    _poll_debug.fifo_temperature_tags = 0;
    _poll_debug.fifo_timestamp_tags = 0;
    _poll_debug.fifo_cfg_change_tags = 0;
    _poll_debug.fifo_accel_ext_tags = 0;
    _poll_debug.fifo_gyro_ext_tags = 0;
    _poll_debug.fifo_other_tags = 0;
    _poll_debug.fifo_recent_raw_tags = 0;
    _poll_debug.gap_over_1ms = 0;
    _poll_debug.gap_over_5ms = 0;
    _poll_debug.gap_over_10ms = 0;
    _poll_debug.max_gap_us = 0;
    _poll_debug.last_report_ms = now_ms;
#else
    (void)now_us;
#endif
}

bool AP_InertialSensor_LSM6DSV::prepare_accel_mode(const uint8_t accel_instance_hint)
{
    _accel_route = AccelRoute::Primary;
    _accel_output_reg = LSM6DSV_REG_OUTX_L_A;
    _accel_status_reg = LSM6DSV_REG_STATUS;
    _accel_ready_mask = LSM6DSV_STATUS_XLDA;
    _accel_scale = _variant_info->accel_scale;
    _accel_range_g = 16.0f;
    _clip_limit = (16.0f - 0.5f) * GRAVITY_MSS;

    {
        WITH_SEMAPHORE(_dev->get_semaphore());
        if (!write_register(LSM6DSV_REG_FIFO_CTRL1, 0x00, true) ||
            !write_register(LSM6DSV_REG_FIFO_CTRL2, 0x00, true) ||
            !write_register(LSM6DSV_REG_FIFO_CTRL3, LSM6DSV_FIFO_CTRL3_BDR_DISABLED, true) ||
            !write_register(LSM6DSV_REG_FIFO_CTRL4, LSM6DSV_FIFO_CTRL4_MODE_BYPASS, true)) {
            return false;
        }

        if (!configure_primary_fifo()) {
            return false;
        }
    }

    if (_variant_info->has_high_g) {
        const uint8_t mode = high_g_mode_for_instance(accel_instance_hint);
        if (mode == 0) {
            return true;
        }

        const float accel_scale = lsm6dsv_high_g_scale_for_mode(mode);
        const float accel_range_g = lsm6dsv_high_g_range_for_mode(mode);
        if (is_zero(accel_scale) || is_zero(accel_range_g)) {
            return false;
        }

        WITH_SEMAPHORE(_dev->get_semaphore());
        if (!configure_high_g_accel(mode)) {
            return false;
        }

        _accel_route = AccelRoute::HighG;
        _accel_output_reg = LSM6DSV_REG_OUTX_L_A_HG;
        _accel_status_reg = LSM6DSV_REG_STATUS;
        _accel_ready_mask = LSM6DSV_STATUS_XLHGDA;
        _accel_scale = accel_scale;
        _accel_range_g = accel_range_g;
        _clip_limit = (accel_range_g - 0.5f) * GRAVITY_MSS;
        return true;
    }

    if (!_variant_info->has_dual_channel || !dual_channel_mode_for_instance(accel_instance_hint)) {
        return true;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!configure_dual_channel_accel()) {
        return false;
    }

    _accel_route = AccelRoute::DualChannel;
    _accel_output_reg = LSM6DSV_REG_OUTX_L_A_HG;
    _accel_status_reg = LSM6DSV_REG_UI_STATUS_OIS;
    _accel_ready_mask = LSM6DSV_STATUS_XLDA_OIS;
    _accel_scale = _variant_info->dual_channel_accel_scale;
    _accel_range_g = _variant_info->dual_channel_accel_range_g;
    _clip_limit = (_accel_range_g - 0.5f) * GRAVITY_MSS;
    return true;
}

bool AP_InertialSensor_LSM6DSV::wait_for_data_ready()
{
    for (uint8_t i = 0; i < LSM6DSV_DATA_READY_TIMEOUT_MS; i++) {
        uint8_t gyro_status = 0;
        uint8_t accel_status = 0;
        hal.scheduler->delay(1);
        if (!read_registers(LSM6DSV_REG_STATUS, &gyro_status, 1)) {
            continue;
        }
        if (_accel_status_reg == LSM6DSV_REG_STATUS) {
            accel_status = gyro_status;
        } else if (!read_registers(_accel_status_reg, &accel_status, 1)) {
            continue;
        }
        if ((gyro_status & LSM6DSV_STATUS_GDA) != 0 &&
            (accel_status & _accel_ready_mask) != 0) {
            return true;
        }
    }

    return false;
}

bool AP_InertialSensor_LSM6DSV::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    return _dev->read_registers(reg, data, len);
}

bool AP_InertialSensor_LSM6DSV::write_register(uint8_t reg, uint8_t value, bool checked)
{
    return _dev->write_register(reg, value, checked);
}

bool AP_InertialSensor_LSM6DSV::fetch_primary_sample(SampleFrame &sample)
{
    // Primary mode is the planned first FIFO landing zone, so keep the current
    // burst-read path isolated from the other route-specific fetch logic.
    RawSampleWithTemp raw{};
    if (!read_registers(LSM6DSV_REG_OUT_TEMP_L, reinterpret_cast<uint8_t *>(&raw), sizeof(raw))) {
        return false;
    }

    sample.raw_temp = int16_t(le16toh(raw.temp));
    sample.has_raw_temp = true;

    sample.gyro = Vector3f{
        float(int16_t(le16toh(raw.gyro[0]))) * _gyro_scale,
        float(int16_t(le16toh(raw.gyro[1]))) * _gyro_scale,
        float(int16_t(le16toh(raw.gyro[2]))) * _gyro_scale,
    };

    sample.accel = Vector3f{
        float(int16_t(le16toh(raw.accel[0]))) * _accel_scale,
        float(int16_t(le16toh(raw.accel[1]))) * _accel_scale,
        float(int16_t(le16toh(raw.accel[2]))) * _accel_scale,
    };

    return true;
}

bool AP_InertialSensor_LSM6DSV::fetch_dual_channel_sample(SampleFrame &sample)
{
    RawAxes raw_gyro{};
    RawAxes raw_accel{};
    if (!read_registers(LSM6DSV_REG_OUTX_L_G, reinterpret_cast<uint8_t *>(&raw_gyro), sizeof(raw_gyro)) ||
        !read_registers(_accel_output_reg, reinterpret_cast<uint8_t *>(&raw_accel), sizeof(raw_accel))) {
        return false;
    }

    sample.gyro = Vector3f{
        float(int16_t(le16toh(raw_gyro.axis[0]))) * _gyro_scale,
        float(int16_t(le16toh(raw_gyro.axis[1]))) * _gyro_scale,
        float(int16_t(le16toh(raw_gyro.axis[2]))) * _gyro_scale,
    };

    sample.accel = Vector3f{
        float(int16_t(le16toh(raw_accel.axis[0]))) * _accel_scale,
        float(int16_t(le16toh(raw_accel.axis[1]))) * _accel_scale,
        float(int16_t(le16toh(raw_accel.axis[2]))) * _accel_scale,
    };

    return true;
}

bool AP_InertialSensor_LSM6DSV::fetch_high_g_sample(SampleFrame &sample)
{
    RawAxes raw_gyro{};
    RawAxes raw_accel{};
    if (!read_registers(LSM6DSV_REG_OUTX_L_G, reinterpret_cast<uint8_t *>(&raw_gyro), sizeof(raw_gyro)) ||
        !read_registers(_accel_output_reg, reinterpret_cast<uint8_t *>(&raw_accel), sizeof(raw_accel))) {
        return false;
    }

    sample.gyro = Vector3f{
        float(int16_t(le16toh(raw_gyro.axis[0]))) * _gyro_scale,
        float(int16_t(le16toh(raw_gyro.axis[1]))) * _gyro_scale,
        float(int16_t(le16toh(raw_gyro.axis[2]))) * _gyro_scale,
    };

    sample.accel = Vector3f{
        float(int16_t(le16toh(raw_accel.axis[0]))) * _accel_scale,
        float(int16_t(le16toh(raw_accel.axis[1]))) * _accel_scale,
        float(int16_t(le16toh(raw_accel.axis[2]))) * _accel_scale,
    };

    return true;
}

bool AP_InertialSensor_LSM6DSV::read_sample(SampleFrame &sample)
{
    sample.raw_temp = 0;
    sample.has_raw_temp = false;

    if (_accel_route == AccelRoute::Primary) {
        return fetch_primary_sample(sample);
    }

    if (_accel_route == AccelRoute::DualChannel) {
        return fetch_dual_channel_sample(sample);
    }

    return fetch_high_g_sample(sample);
}

bool AP_InertialSensor_LSM6DSV::read_status_registers(uint8_t &gyro_status, uint8_t &accel_status, uint32_t now_us)
{
    gyro_status = 0;
    accel_status = 0;

    if (!read_registers(LSM6DSV_REG_STATUS, &gyro_status, 1)) {
#if LSM6DSV_TIMING_DEBUG_ENABLED
        _poll_debug.read_failures++;
        maybe_log_poll_debug(now_us);
#endif
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return false;
    }

    if (_accel_status_reg == LSM6DSV_REG_STATUS) {
        accel_status = gyro_status;
        return true;
    }

    if (!read_registers(_accel_status_reg, &accel_status, 1)) {
#if LSM6DSV_TIMING_DEBUG_ENABLED
        _poll_debug.read_failures++;
        maybe_log_poll_debug(now_us);
#endif
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return false;
    }

    return true;
}

void AP_InertialSensor_LSM6DSV::update_dual_channel_ready_stats(bool gyro_ready, bool accel_ready)
{
    if (_accel_route != AccelRoute::DualChannel) {
        return;
    }

    if (gyro_ready && accel_ready) {
        _poll_debug.dualc_both_ready++;
    } else if (gyro_ready) {
        _poll_debug.dualc_gyro_only_ready++;
    } else if (accel_ready) {
        _poll_debug.dualc_accel_only_ready++;
    } else {
        _poll_debug.dualc_neither_ready++;
    }
}

bool AP_InertialSensor_LSM6DSV::sample_ready_for_route(uint8_t gyro_status, uint8_t accel_status, uint32_t now_us)
{
    const bool gyro_ready = (gyro_status & LSM6DSV_STATUS_GDA) != 0;
    const bool accel_ready = (accel_status & _accel_ready_mask) != 0;

    update_dual_channel_ready_stats(gyro_ready, accel_ready);

    if (_accel_route == AccelRoute::HighG || (gyro_ready && accel_ready)) {
        return true;
    }

#if LSM6DSV_TIMING_DEBUG_ENABLED
    _poll_debug.ready_skips++;
    if (!gyro_ready) {
        _poll_debug.gyro_not_ready++;
    }
    if (!accel_ready) {
        _poll_debug.accel_not_ready++;
    }
    maybe_log_poll_debug(now_us);
#endif
    return false;
}

bool AP_InertialSensor_LSM6DSV::fetch_current_sample(SampleFrame &sample, uint32_t now_us)
{
    if (read_sample(sample)) {
        return true;
    }

#if LSM6DSV_TIMING_DEBUG_ENABLED
    _poll_debug.read_failures++;
    maybe_log_poll_debug(now_us);
#endif
    _inc_accel_error_count(accel_instance);
    _inc_gyro_error_count(gyro_instance);
    return false;
}

bool AP_InertialSensor_LSM6DSV::fetch_polling_frame(PollingFrame &frame, uint32_t now_us)
{
    frame.gyro_status = 0;
    frame.accel_status = 0;

    if (!read_status_registers(frame.gyro_status, frame.accel_status, now_us)) {
        return false;
    }

    if (!sample_ready_for_route(frame.gyro_status, frame.accel_status, now_us)) {
        return false;
    }

    return fetch_current_sample(frame.sample, now_us);
}

bool AP_InertialSensor_LSM6DSV::fetch_source_frame(SourceFrame &frame, uint32_t now_us)
{
    // Polling path only; FIFO path is handled by drain_fifo() in poll_data()
    PollingFrame polling_frame{};
    if (!fetch_polling_frame(polling_frame, now_us)) {
        return false;
    }
    frame.sample = polling_frame.sample;
    frame.gyro_status = polling_frame.gyro_status;
    frame.accel_status = polling_frame.accel_status;
    return true;
}

bool AP_InertialSensor_LSM6DSV::read_fifo_status(FifoFrame &frame, uint32_t now_us)
{
    uint8_t fifo_status[2] {};
    if (!read_registers(LSM6DSV_REG_FIFO_STATUS1, fifo_status, sizeof(fifo_status))) {
#if LSM6DSV_TIMING_DEBUG_ENABLED
        _poll_debug.read_failures++;
        maybe_log_poll_debug(now_us);
#endif
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return false;
    }

    frame.unread_words = fifo_status[0] |
                         (((fifo_status[1] & LSM6DSV_FIFO_STATUS2_DIFF_FIFO_8) != 0U) ? 0x100U : 0U);

    return true;
}

bool AP_InertialSensor_LSM6DSV::read_fifo_words_block(const uint16_t n_words, uint32_t now_us)
{
    if (_fifo_buffer == nullptr || n_words == 0 || n_words > LSM6DSV_FIFO_BURST_WORDS) {
        return false;
    }

    _fifo_buffer[0] = LSM6DSV_REG_FIFO_DATA_OUT_TAG | LSM6DSV_SPI_READ_FLAG;
    memset(_fifo_buffer + 1, 0, n_words * sizeof(RawFifoWord));
    if (!_dev->transfer_fullduplex(_fifo_buffer, n_words * sizeof(RawFifoWord) + 1)) {
#if LSM6DSV_TIMING_DEBUG_ENABLED
        _poll_debug.read_failures++;
        maybe_log_poll_debug(now_us);
#endif
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return false;
    }

    return true;
}

bool AP_InertialSensor_LSM6DSV::consume_fifo_word(FifoFrame &frame, SampleFrame &sample, const uint8_t *raw_word)
{
    RawFifoWord raw{};
    memcpy(&raw, raw_word, sizeof(raw));

    frame.tag = decode_fifo_tag(raw.tag);
    frame.tag_count = decode_fifo_tag_count(raw.tag);
    _poll_debug.fifo_words_read++;
    _poll_debug.fifo_recent_raw_tags = (_poll_debug.fifo_recent_raw_tags << 8) | raw.tag;

    switch (frame.tag) {
    case FifoTag::GyroNC:
        _poll_debug.fifo_gyro_nc_tags++;
        break;
    case FifoTag::AccelNC:
        _poll_debug.fifo_accel_nc_tags++;
        break;
    case FifoTag::Temperature:
        _poll_debug.fifo_temperature_tags++;
        break;
    case FifoTag::Timestamp:
        _poll_debug.fifo_timestamp_tags++;
        break;
    case FifoTag::CfgChange:
        _poll_debug.fifo_cfg_change_tags++;
        break;
    case FifoTag::AccelNC_T2:
    case FifoTag::AccelNC_T1:
    case FifoTag::Accel2xC:
    case FifoTag::Accel3xC:
        _poll_debug.fifo_accel_ext_tags++;
        break;
    case FifoTag::GyroNC_T2:
    case FifoTag::GyroNC_T1:
    case FifoTag::Gyro2xC:
    case FifoTag::Gyro3xC:
        _poll_debug.fifo_gyro_ext_tags++;
        break;
    case FifoTag::Empty:
    case FifoTag::RouteExt:
    case FifoTag::Unsupported:
        _poll_debug.fifo_other_tags++;
        break;
    }

    if (!fifo_tag_supported_for_primary(frame.tag)) {
        return true;
    }

    const Vector3f axes{
        float(int16_t(le16toh(raw.axis[0]))),
        float(int16_t(le16toh(raw.axis[1]))),
        float(int16_t(le16toh(raw.axis[2]))),
    };

    switch (frame.tag) {
    case FifoTag::GyroNC:
        sample.gyro = axes * _gyro_scale;
        break;
    case FifoTag::AccelNC:
        sample.accel = axes * _accel_scale;
        break;
    default:
        return false;
    }

    return true;
}

uint16_t AP_InertialSensor_LSM6DSV::drain_fifo(uint32_t now_us)
{
    if (!fifo_supported_for_current_route()) {
        return 0;
    }

    FifoFrame frame{};
    frame.tag = FifoTag::Empty;
    frame.unread_words = 0;
    frame.tag_count = 0;
    frame.source.gyro_status = 0;
    frame.source.accel_status = 0;
    frame.source.sample.raw_temp = 0;
    frame.source.sample.has_raw_temp = false;
    frame.source.sample.gyro.zero();
    frame.source.sample.accel.zero();

    if (!read_fifo_status(frame, now_us)) {
        return 0;
    }

    if (frame.unread_words == 0) {
        return 0;
    }

    uint16_t samples_published = 0;
    uint16_t drained = 0;

    while (drained < frame.unread_words && drained < LSM6DSV_FIFO_MAX_DRAIN_WORDS) {
        const uint16_t remaining = MIN(uint16_t(frame.unread_words - drained),
                                       uint16_t(LSM6DSV_FIFO_MAX_DRAIN_WORDS - drained));
        const uint16_t block_words = MIN(remaining, LSM6DSV_FIFO_BURST_WORDS);

        if (!read_fifo_words_block(block_words, now_us)) {
            break;
        }

        const uint8_t *raw_word = _fifo_buffer + 1;
        for (uint16_t i = 0; i < block_words; i++, raw_word += sizeof(RawFifoWord)) {
            if (!consume_fifo_word(frame, frame.source.sample, raw_word)) {
                return samples_published;
            }
            drained++;
            if (frame.tag == FifoTag::GyroNC) {
                publish_gyro_sample(frame.source.sample);
                samples_published++;
                frame.source.sample.gyro.zero();
            } else if (frame.tag == FifoTag::AccelNC) {
                publish_accel_sample(frame.source.sample);
                frame.source.sample.accel.zero();
            }
        }
    }

    if (samples_published > 0) {
        _dev->adjust_periodic_callback(periodic_handle, _backend_period_us);
    }

    return samples_published;
}

void AP_InertialSensor_LSM6DSV::publish_gyro_sample(SampleFrame &sample)
{
    _rotate_and_correct_gyro(gyro_instance, sample.gyro);
    _notify_new_gyro_raw_sample(gyro_instance, sample.gyro);
}

void AP_InertialSensor_LSM6DSV::publish_accel_sample(SampleFrame &sample)
{
    _rotate_and_correct_accel(accel_instance, sample.accel);
    _notify_new_accel_raw_sample(accel_instance, sample.accel);
    update_temperature(LSM6DSV_STATUS_GDA, nullptr);
}

void AP_InertialSensor_LSM6DSV::publish_current_sample(SampleFrame &sample, uint8_t gyro_status)
{
    _rotate_and_correct_accel(accel_instance, sample.accel);
    _rotate_and_correct_gyro(gyro_instance, sample.gyro);
    _notify_new_accel_raw_sample(accel_instance, sample.accel);
    _notify_new_gyro_raw_sample(gyro_instance, sample.gyro);
    update_temperature(gyro_status, sample.has_raw_temp ? &sample.raw_temp : nullptr);
}

void AP_InertialSensor_LSM6DSV::check_register_monitor()
{
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
#if LSM6DSV_TIMING_DEBUG_ENABLED
        _poll_debug.register_failures++;
#endif
    }
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

void AP_InertialSensor_LSM6DSV::update_temperature(const uint8_t status, const int16_t *raw_temp)
{
    if (_temperature_counter++ < LSM6DSV_TEMPERATURE_UPDATE_INTERVAL) {
        return;
    }
    _temperature_counter = 0;

    int16_t temperature_raw = 0;
    if (raw_temp != nullptr) {
        temperature_raw = *raw_temp;
    } else {
        const bool primary_fifo_temp_poll =
            active_sample_source() == SampleSourceMode::FIFO && _accel_route == AccelRoute::Primary;
        if (!primary_fifo_temp_poll && (status & LSM6DSV_STATUS_TDA) == 0) {
            return;
        }
        uint8_t tbuf[2];
        if (!read_registers(LSM6DSV_REG_OUT_TEMP_L, tbuf, sizeof(tbuf))) {
            _inc_accel_error_count(accel_instance);
            return;
        }
        temperature_raw = int16_t(uint16_t(tbuf[0] | (tbuf[1] << 8)));
    }

    const float temp_degc = LSM6DSV_TEMPERATURE_ZERO_C + temperature_raw / LSM6DSV_TEMPERATURE_SENSITIVITY;
    _publish_temperature(accel_instance, temp_degc);
#if LSM6DSV_TIMING_DEBUG_ENABLED
    _poll_debug.temp_updates++;
#endif
}

void AP_InertialSensor_LSM6DSV::poll_data()
{
#if LSM6DSV_TIMING_DEBUG_ENABLED
    const uint32_t now_us = AP_HAL::micros();
    if (_poll_debug.last_poll_us != 0) {
        const uint32_t callback_gap_us = now_us - _poll_debug.last_poll_us;
        _poll_debug.max_gap_us = MAX(_poll_debug.max_gap_us, callback_gap_us);
        if (callback_gap_us > 1000U) {
            _poll_debug.gap_over_1ms++;
        }
        if (callback_gap_us > 5000U) {
            _poll_debug.gap_over_5ms++;
        }
        if (callback_gap_us > 10000U) {
            _poll_debug.gap_over_10ms++;
        }
    }
    _poll_debug.last_poll_us = now_us;
    _poll_debug.callbacks++;
#else
    const uint32_t now_us = 0;
#endif

    if (active_sample_source() == SampleSourceMode::FIFO) {
        const uint16_t n_published = drain_fifo(now_us);
        check_register_monitor();
#if LSM6DSV_TIMING_DEBUG_ENABLED
        _poll_debug.publishes += n_published;
        maybe_log_poll_debug(now_us);
#endif
        return;
    }

    // Polling path (unchanged)
    SourceFrame frame{};
    if (!fetch_source_frame(frame, now_us)) {
        return;
    }

    publish_current_sample(frame.sample, frame.gyro_status);
    check_register_monitor();
#if LSM6DSV_TIMING_DEBUG_ENABLED
    _poll_debug.publishes++;
    maybe_log_poll_debug(now_us);
#endif
}
