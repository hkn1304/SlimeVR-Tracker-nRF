/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "BHI385.h"

#include "bhi385.h"
#include "bhi385_parse.h"
#include "bhi385_virtual_sensor_conf_param.h"
#include "bhi385_event_data.h"

LOG_MODULE_REGISTER(BHI385, LOG_LEVEL_DBG);

extern const uint8_t bhi385_firmware_image[];
extern const uint32_t bhi385_firmware_image_size;

#define BHI385_WORK_BUF_SIZE 2048

static struct bhi385_dev bhy;
static uint8_t bhi385_work_buf[BHI385_WORK_BUF_SIZE];
static float bhi385_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

void bhi385_get_quaternion(float q[4])
{
	q[0] = bhi385_quat[0];
	q[1] = bhi385_quat[1];
	q[2] = bhi385_quat[2];
	q[3] = bhi385_quat[3];
}

static int8_t bhi385_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	return ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, reg_addr, reg_data, (uint16_t)length)
		? BHI385_E_IO : BHI385_OK;
}

static int8_t bhi385_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	return ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, reg_addr, reg_data, (uint16_t)length)
		? BHI385_E_IO : BHI385_OK;
}

static void bhi385_delay_us_cb(uint32_t period_us, void *intf_ptr)
{
	k_usleep(period_us);
}

static void on_rotation_vector(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
	if (callback_info->data_size != 11)
		return;
	struct bhi385_event_data_quaternion data;
	bhi385_event_data_parse_quaternion(callback_info->data_ptr, &data);
	bhi385_quat[0] = data.w / 16384.0f;
	bhi385_quat[1] = data.x / 16384.0f;
	bhi385_quat[2] = data.y / 16384.0f;
	bhi385_quat[3] = data.z / 16384.0f;
}

static int bhi385_drv_init(float clock_rate, float accel_time, float gyro_time,
			    float *accel_actual_time, float *gyro_actual_time)
{
	int8_t rslt;
	uint8_t boot_status = 0;

	rslt = bhi385_init(BHI385_I2C_INTERFACE,
			   bhi385_i2c_read, bhi385_i2c_write, bhi385_delay_us_cb,
			   256, NULL, &bhy);
	if (rslt != BHI385_OK) {
		LOG_ERR("bhi385_init failed: %d", rslt);
		return -EIO;
	}

	for (int i = 0; i < 100; i++) {
		rslt = bhi385_get_boot_status(&boot_status, &bhy);
		if (rslt == BHI385_OK && (boot_status & BHI385_BST_HOST_INTERFACE_READY))
			break;
		k_msleep(10);
	}
	if (!(boot_status & BHI385_BST_HOST_INTERFACE_READY)) {
		LOG_ERR("BHI385 host interface not ready (0x%02X)", boot_status);
		return -ETIMEDOUT;
	}

	rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, bhi385_firmware_image_size, &bhy);
	if (rslt != BHI385_OK) {
		LOG_ERR("Firmware upload failed: %d", rslt);
		return -EIO;
	}

	rslt = bhi385_boot_from_ram(&bhy);
	if (rslt != BHI385_OK) {
		LOG_ERR("Boot from RAM failed: %d", rslt);
		return -EIO;
	}

	k_msleep(200);

	rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_RV, on_rotation_vector, NULL, &bhy);
	if (rslt != BHI385_OK) {
		LOG_ERR("Failed to register RV callback: %d", rslt);
		return -EIO;
	}

	bhi385_get_and_process_fifo(bhi385_work_buf, BHI385_WORK_BUF_SIZE, &bhy);
	bhi385_update_virtual_sensor_list(&bhy);

	struct bhi385_virtual_sensor_conf_param_conf sensor_conf = {
		.sample_rate = 100.0f,
		.latency = 0
	};
	rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_RV, &sensor_conf, &bhy);
	if (rslt != BHI385_OK) {
		LOG_ERR("Failed to configure RV sensor: %d", rslt);
		return -EIO;
	}

	LOG_INF("BHI385 9DOF (NDOF+BMM350C) ready at 100Hz");
	*accel_actual_time = 0.01f;
	*gyro_actual_time = 0.01f;
	return 0;
}

static void bhi385_drv_shutdown(void)
{
	bhi385_soft_reset(&bhy);
}

static void bhi385_drv_update_fs(float accel_range, float gyro_range,
				  float *accel_actual_range, float *gyro_actual_range)
{
	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

static int bhi385_drv_update_odr(float accel_time, float gyro_time,
				  float *accel_actual_time, float *gyro_actual_time)
{
	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;
	return 0;
}

static uint16_t bhi385_drv_fifo_read(uint8_t *buf, uint16_t buf_size)
{
	bhi385_get_and_process_fifo(bhi385_work_buf, BHI385_WORK_BUF_SIZE, &bhy);
	return 1;
}

static int bhi385_drv_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	a[0] = a[1] = a[2] = 0.0f;
	g[0] = g[1] = g[2] = 0.0f;
	return 0;
}

static void bhi385_drv_accel_read(float a[3])
{
	a[0] = a[1] = a[2] = 0.0f;
}

static void bhi385_drv_gyro_read(float g[3])
{
	g[0] = g[1] = g[2] = 0.0f;
}

static int bhi385_drv_temp_read(float *t)
{
	return -1;
}

static uint8_t bhi385_drv_setup_DRDY(uint16_t threshold)
{
	return 0;
}

static uint8_t bhi385_drv_setup_WOM(void)
{
	return 0;
}

static int bhi385_drv_ext_setup(void)
{
	return -1;
}

static int bhi385_drv_ext_passthrough(bool enable)
{
	return -1;
}

const sensor_imu_t sensor_imu_bhi385 = {
	bhi385_drv_init,
	bhi385_drv_shutdown,
	bhi385_drv_update_fs,
	bhi385_drv_update_odr,
	bhi385_drv_fifo_read,
	bhi385_drv_fifo_process,
	bhi385_drv_accel_read,
	bhi385_drv_gyro_read,
	bhi385_drv_temp_read,
	bhi385_drv_setup_DRDY,
	bhi385_drv_setup_WOM,
	bhi385_drv_ext_setup,
	bhi385_drv_ext_passthrough
};
