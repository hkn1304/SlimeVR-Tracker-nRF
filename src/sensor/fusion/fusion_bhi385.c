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
#include "fusion_bhi385.h"
#include "sensor/imu/BHI385.h"

static void bhi385_fusion_init(float g_time, float a_time, float m_time) {}
static void bhi385_fusion_load(const void *data) {}
static void bhi385_fusion_save(void *data) {}
static void bhi385_fusion_update_gyro(float *g, float time) {}
static void bhi385_fusion_update_accel(float *a, float time) {}
static void bhi385_fusion_update_mag(float *m, float time) {}
static void bhi385_fusion_update(float *g, float *a, float *m, float time) {}

static void bhi385_fusion_get_gyro_bias(float *g_off)
{
	g_off[0] = g_off[1] = g_off[2] = 0.0f;
}

static void bhi385_fusion_set_gyro_bias(float *g_off) {}
static void bhi385_fusion_update_gyro_sanity(float *g, float *m) {}

static int bhi385_fusion_get_gyro_sanity(void)
{
	return 1;
}

static void bhi385_fusion_get_lin_a(float *lin_a)
{
	lin_a[0] = lin_a[1] = lin_a[2] = 0.0f;
}

static void bhi385_fusion_get_quat(float *q)
{
	bhi385_get_quaternion(q);
}

const sensor_fusion_t sensor_fusion_bhi385 = {
	bhi385_fusion_init,
	bhi385_fusion_load,
	bhi385_fusion_save,
	bhi385_fusion_update_gyro,
	bhi385_fusion_update_accel,
	bhi385_fusion_update_mag,
	bhi385_fusion_update,
	bhi385_fusion_get_gyro_bias,
	bhi385_fusion_set_gyro_bias,
	bhi385_fusion_update_gyro_sanity,
	bhi385_fusion_get_gyro_sanity,
	bhi385_fusion_get_lin_a,
	bhi385_fusion_get_quat
};
