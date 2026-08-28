#include <stdint.h>
#include <string.h>

#include "vl53lx_api.h"
#include "vl53lx_def.h"
#include "vl53lx_platform.h"

static VL53LX_Dev_t bottom_tof;
static int bottom_initialized = 0;

static int return_if_error(VL53LX_Error status)
{
    return status == VL53LX_ERROR_NONE ? 0 : (int)status;
}

int rust_fly_tof_bottom_init(void)
{
    memset(&bottom_tof, 0, sizeof(bottom_tof));
    bottom_tof.comms_speed_khz = 400;
    bottom_tof.i2c_slave_address = 0x29;

    int status = return_if_error(VL53LX_WaitDeviceBooted(&bottom_tof));
    if (status != 0) {
        return status;
    }

    status = return_if_error(VL53LX_DataInit(&bottom_tof));
    if (status != 0) {
        return status;
    }

    status = return_if_error(VL53LX_SetDistanceMode(&bottom_tof, VL53LX_DISTANCEMODE_MEDIUM));
    if (status != 0) {
        return status;
    }

    status = return_if_error(VL53LX_SetMeasurementTimingBudgetMicroSeconds(&bottom_tof, 33000));
    if (status != 0) {
        return status;
    }

    status = return_if_error(VL53LX_ClearInterruptAndStartMeasurement(&bottom_tof));
    if (status != 0) {
        return status;
    }

    status = return_if_error(VL53LX_WaitMs(&bottom_tof, 100));
    if (status != 0) {
        return status;
    }

    status = return_if_error(VL53LX_StartMeasurement(&bottom_tof));
    if (status != 0) {
        return status;
    }

    bottom_initialized = 1;
    return 0;
}

int rust_fly_tof_bottom_model_id(uint8_t *model_id, uint8_t *module_type)
{
    if (model_id == 0 || module_type == 0) {
        return (int)VL53LX_ERROR_INVALID_PARAMS;
    }

    int status = return_if_error(VL53LX_RdByte(&bottom_tof, 0x010f, model_id));
    if (status != 0) {
        return status;
    }

    return return_if_error(VL53LX_RdByte(&bottom_tof, 0x0110, module_type));
}

int rust_fly_tof_bottom_read_range(
    int16_t *range_mm,
    uint8_t *range_status,
    uint8_t *objects)
{
    if (!bottom_initialized || range_mm == 0 || range_status == 0 || objects == 0) {
        return (int)VL53LX_ERROR_INVALID_PARAMS;
    }

    uint8_t data_ready = 0;
    int status = return_if_error(VL53LX_GetMeasurementDataReady(&bottom_tof, &data_ready));
    if (status != 0) {
        return status;
    }

    if (data_ready == 0) {
        *range_mm = 0;
        *range_status = VL53LX_RANGESTATUS_NONE;
        *objects = 0;
        return 0;
    }

    VL53LX_MultiRangingData_t data;
    memset(&data, 0, sizeof(data));

    status = return_if_error(VL53LX_GetMultiRangingData(&bottom_tof, &data));
    if (status != 0) {
        return status;
    }

    *objects = data.NumberOfObjectsFound;
    *range_mm = 0;
    *range_status = VL53LX_RANGESTATUS_NONE;

    for (uint8_t index = 0; index < data.NumberOfObjectsFound; index++) {
        uint8_t status_for_object = data.RangeData[index].RangeStatus;
        int16_t range_for_object = data.RangeData[index].RangeMilliMeter;

        if (index == 0) {
            *range_status = status_for_object;
            *range_mm = range_for_object;
        }

        if (status_for_object == VL53LX_RANGESTATUS_RANGE_VALID && range_for_object > *range_mm) {
            *range_status = status_for_object;
            *range_mm = range_for_object;
        }
    }

    return return_if_error(VL53LX_ClearInterruptAndStartMeasurement(&bottom_tof));
}
