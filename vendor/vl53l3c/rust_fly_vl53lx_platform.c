#include <stdint.h>
#include <string.h>

#include "vl53lx_platform.h"

#define RUST_FLY_I2C_BUFFER_SIZE 1024

extern int32_t rust_fly_tof_i2c_write(uint8_t address, const uint8_t *data, uint32_t len);
extern int32_t rust_fly_tof_i2c_write_read(
    uint8_t address,
    const uint8_t *write_data,
    uint32_t write_len,
    uint8_t *read_data,
    uint32_t read_len);
extern void rust_fly_tof_delay_us(uint32_t delay_us);
extern uint32_t rust_fly_tof_millis(void);

static uint8_t i2c_buffer[RUST_FLY_I2C_BUFFER_SIZE];

static VL53LX_Error i2c_status_to_vl53(int32_t status)
{
    return status == 0 ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

int vl53lx_i2c_init(void)
{
    return 0;
}

VL53LX_Error VL53LX_CommsInitialise(
    VL53LX_Dev_t *pdev,
    uint8_t comms_type,
    uint16_t comms_speed_khz)
{
    pdev->comms_type = comms_type;
    pdev->comms_speed_khz = comms_speed_khz;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev)
{
    (void)pdev;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WriteMulti(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    if (count + 2 > RUST_FLY_I2C_BUFFER_SIZE) {
        return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
    }

    i2c_buffer[0] = (uint8_t)(index >> 8);
    i2c_buffer[1] = (uint8_t)(index & 0xff);
    memcpy(&i2c_buffer[2], pdata, count);

    int32_t status = rust_fly_tof_i2c_write(
        pdev->i2c_slave_address,
        i2c_buffer,
        count + 2);

    return i2c_status_to_vl53(status);
}

VL53LX_Error VL53LX_ReadMulti(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    uint8_t address[2] = {
        (uint8_t)(index >> 8),
        (uint8_t)(index & 0xff),
    };

    int32_t status = rust_fly_tof_i2c_write_read(
        pdev->i2c_slave_address,
        address,
        sizeof(address),
        pdata,
        count);

    return i2c_status_to_vl53(status);
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t data)
{
    return VL53LX_WriteMulti(pdev, index, &data, 1);
}

VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t data)
{
    uint8_t buffer[2] = {
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xff),
    };

    return VL53LX_WriteMulti(pdev, index, buffer, sizeof(buffer));
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t data)
{
    uint8_t buffer[4] = {
        (uint8_t)((data >> 24) & 0xff),
        (uint8_t)((data >> 16) & 0xff),
        (uint8_t)((data >> 8) & 0xff),
        (uint8_t)(data & 0xff),
    };

    return VL53LX_WriteMulti(pdev, index, buffer, sizeof(buffer));
}

VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata)
{
    return VL53LX_ReadMulti(pdev, index, pdata, 1);
}

VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t *pdata)
{
    uint8_t buffer[2] = {0};
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, sizeof(buffer));
    if (status == VL53LX_ERROR_NONE) {
        *pdata = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    }
    return status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t *pdata)
{
    uint8_t buffer[4] = {0};
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, sizeof(buffer));
    if (status == VL53LX_ERROR_NONE) {
        *pdata = ((uint32_t)buffer[0] << 24)
            | ((uint32_t)buffer[1] << 16)
            | ((uint32_t)buffer[2] << 8)
            | (uint32_t)buffer[3];
    }
    return status;
}

VL53LX_Error VL53LX_UpdateByte(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t and_data,
    uint8_t or_data)
{
    uint8_t data = 0;
    VL53LX_Error status = VL53LX_RdByte(pdev, index, &data);
    if (status != VL53LX_ERROR_NONE) {
        return status;
    }

    data = (data & and_data) | or_data;
    return VL53LX_WrByte(pdev, index, data);
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us)
{
    (void)pdev;
    if (wait_us > 0) {
        rust_fly_tof_delay_us((uint32_t)wait_us);
    }
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms)
{
    (void)pdev;
    if (wait_ms > 0) {
        rust_fly_tof_delay_us((uint32_t)wait_ms * 1000);
    }
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = 1000;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
    *ptimer_count = (int32_t)rust_fly_tof_millis();
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_DEV dev, uint32_t *ptime_ms)
{
    (void)dev;
    *ptime_ms = rust_fly_tof_millis();
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    (void)mode;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
    (void)pin;
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
    (void)pin;
    *pvalue = 0;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
    (void)function;
    (void)edge_type;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptDisable(void)
{
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t timeout_ms,
    uint16_t index,
    uint8_t value,
    uint8_t mask,
    uint32_t poll_delay_ms)
{
    uint32_t start_ms = 0;
    uint32_t now_ms = 0;
    uint8_t byte_value = 0;

    VL53LX_GetTickCount(pdev, &start_ms);

    while (1) {
        VL53LX_Error status = VL53LX_RdByte(pdev, index, &byte_value);
        if (status != VL53LX_ERROR_NONE) {
            return status;
        }

        if ((byte_value & mask) == value) {
            return VL53LX_ERROR_NONE;
        }

        VL53LX_GetTickCount(pdev, &now_ms);
        if ((now_ms - start_ms) >= timeout_ms) {
            return VL53LX_ERROR_TIME_OUT;
        }

        if (poll_delay_ms > 0) {
            VL53LX_WaitMs(pdev, (int32_t)poll_delay_ms);
        }
    }
}
