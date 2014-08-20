/*
 * Copyright (c) 2014 Hanspeter Portner (dev@open-music-kontrollers.ch)
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 *     1. The origin of this software must not be misrepresented; you must not
 *     claim that you wrote the original software. If you use this software
 *     in a product, an acknowledgment in the product documentation would be
 *     appreciated but is not required.
 * 
 *     2. Altered source versions must be plainly marked as such, and must not be
 *     misrepresented as being the original software.
 * 
 *     3. This notice may not be removed or altered from any source
 *     distribution.
 */

#include <math.h>

#include <oscpod.h>
#include <eeprom.h>
#include <linalg.h>

#include "calibration_private.h"

// globals
Calibration range;
uint_fast8_t zeroing = 0;
uint_fast8_t calibrating = 0;

uint16_t arr [SENSOR_N];
static Calibration_Point point [SENSOR_N];

uint_fast8_t
range_load(uint_fast8_t pos)
{
	eeprom_bulk_read(eeprom_24LC64, EEPROM_RANGE_OFFSET + pos*EEPROM_RANGE_SIZE,(uint8_t *)&range, sizeof(range));

	return 1;
}

uint_fast8_t
range_reset()
{
	uint_fast8_t i;

	for(i=0; i<SENSOR_N; i++)
	{
		range.Q[i] = 0UL;
		range.Bmin[i] = 0.f;
		range.W[i] = 1.0 / 0xfff;
		range.C[i][0] = 0.f; // ~ cbrt(x)
		range.C[i][1] = 1.f; // ~ sqrt(x)
		range.C[i][2] = 0.f; // ~ x
	}

	return 1;
}

uint_fast8_t
range_save(uint_fast8_t pos)
{
	eeprom_bulk_write(eeprom_24LC64, EEPROM_RANGE_OFFSET + pos*EEPROM_RANGE_SIZE,(uint8_t *)&range, sizeof(range));

	return 1;
}

void
range_calibrate(int16_t *raw)
{
	uint_fast8_t i;

	// do the calibration
	for(i=0; i<SENSOR_N; i++)
	{
		int16_t avg;

		if(zeroing)
		{
			// moving average over 16 samples
			range.Q[i] -= range.Q[i] >> 4;
			range.Q[i] += raw[i];
		}

		if(raw[i] > (avg = arr[i] >> 4) )
		{
			arr[i] -= avg;
			arr[i] += raw[i];
		}
	}
}

// initialize sensor range
void
range_init()
{
	uint_fast8_t i;

	for(i=0; i<SENSOR_N; i++)
	{
		range.Q[i] = 0;
		arr[i] = range.Q[i];
		point[i].state = 1;
	}
}

// calibrate quiescent current
static void
range_update_quiescent()
{
	uint_fast8_t i;

	for(i=0; i<SENSOR_N; i++)
	{
		// final average over last 16 samples
		range.Q[i] >>= 4;

		// reset array to quiescent value
		arr[i] = range.Q[i] << 4;
	}
}

// calibrate threshold
static void
range_update_b0()
{
	uint_fast8_t i;
	for(i=0; i<SENSOR_N; i++)
	{
		arr[i] >>= 4; // average over 16 samples
		uint16_t b = arr[i] - range.Q[i];
		range.Bmin[i] = b;

		// reset thresh to quiescent value
		arr[i] = range.Q[i] << 4;
	}
}

// calibrate distance-magnetic flux relationship curve
static uint_fast8_t
range_update_b1(float y)
{
	uint_fast8_t i;
	uint_fast8_t ret = 0;

	for(i=0; i<SENSOR_N; i++)
	{
		arr[i] >>= 4; // average over 16 samples
		uint16_t b = arr[i] - range.Q[i];
		
		switch(point[i].state)
		{
			case 1:
				if(y <= 0.f) // check for increasing y
					goto exit;
				point[i].y1 = y;
				point[i].B1 = b;
				point[i].state++;
				break;
			case 2:
				if(y <= point[i].y1) // check for increasing y
					goto exit;
				point[i].y2 = y;
				point[i].B2 = b;
				point[i].state++;
				break;
			case 3:
				if(y <= point[i].y2) // check for increasing y
					goto exit;
				point[i].y3 = y;
				point[i].B3 = b;
				point[i].state++;
				break;
		}
	}
	
	ret = 1;

exit:

	// reset arr to quiescent values
	for(i=0; i<SENSOR_N; i++)
		arr[i] = range.Q[i] << 4;

	return ret;
}

// calibrate amplification and sensitivity
static uint_fast8_t
range_update_b2()
{
	uint_fast8_t i;
	uint_fast8_t ret = 0;

	for(i=0; i<SENSOR_N; i++)
	{
		arr[i] >>= 4; // average over 16 samples
		uint16_t b = arr[i] - range.Q[i];

		if(point[i].state == 4)
		{
			range.W[i] = 1.f / (b - point[i].B0);

			// skip for pressure sensor
			if(i != SENSOR_N-1)
			{
				// normalize
				point[i].B1 = (point[i].B1 - point[i].B0) * range.W[i];
				point[i].B2 = (point[i].B2 - point[i].B0) * range.W[i];
				point[i].B3 = (point[i].B3 - point[i].B0) * range.W[i];

				double C [3];
				linalg_least_squares_cubic(point[i].B1, point[i].y1, point[i].B2, point[i].y2, point[i].B3, point[i].y3, &C[0], &C[1], &C[2]);
				range.C[i][0] =(float)C[0];
				range.C[i][1] =(float)C[1];
				range.C[i][2] =(float)C[2];
			}

			ret = 1;
		}
		else
			goto exit;
	}

exit:

	// reset arr to quiescent values
	for(i=0; i<SENSOR_N; i++)
		arr[i] = range.Q[i] << 4;

	return ret;
}

/*
 * Config
 */

static uint_fast8_t
_calibration_start(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);

	range_init();

	// enable calibration
	zeroing = 1;
	calibrating = 1;

	size = CONFIG_SUCCESS("is", uuid, path);
	CONFIG_SEND(size);

	return 1;
}

static uint_fast8_t
_calibration_zero(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);

	if(calibrating)
	{
		// update new range
		zeroing = 0;
		range_update_quiescent();
		size = CONFIG_SUCCESS("is", uuid, path);
	}
	else
		size = CONFIG_FAIL("iss", uuid, path, "not in calibration mode");

	CONFIG_SEND(size);

	return 1;
}

static uint_fast8_t
_calibration_min(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);

	if(calibrating)
	{
		// update new range
		range_update_b0();
		size = CONFIG_SUCCESS("is", uuid, path);
	}
	else
		size = CONFIG_FAIL("iss", uuid, path, "not in calibration mode");

	CONFIG_SEND(size);

	return 1;
}

static uint_fast8_t
_calibration_mid(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);

	if(calibrating)
	{
		float y;
		buf_ptr = osc_get_float(buf_ptr, &y);

		// update mid range
		if(range_update_b1(y))
			size = CONFIG_SUCCESS("is", uuid, path);
		else
			size = CONFIG_FAIL("iss", uuid, path, "vicinity must be increasing for five-point curve-fit");
	}
	else
		size = CONFIG_FAIL("iss", uuid, path, "not in calibration mode");
		
	CONFIG_SEND(size);

	return 1;
}

static uint_fast8_t
_calibration_max(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);

	if(calibrating)
	{
		// update max range
		if(range_update_b2())
		{
			calibrating = 0;
			size = CONFIG_SUCCESS("is", uuid, path);
		}
		else
			size = CONFIG_FAIL("iss", uuid, path, "not all points given for five-point curve-fit");
	}
	else
		size = CONFIG_FAIL("iss", uuid, path, "not in calibration mode");

	CONFIG_SEND(size);

	return 1;
}

static uint_fast8_t
_calibration_save(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;
	int32_t pos;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);
	buf_ptr = osc_get_int32(buf_ptr, &pos);

	range_save(pos);
	size = CONFIG_SUCCESS("is", uuid, path);
	CONFIG_SEND(size);

	return 1;
}

static uint_fast8_t
_calibration_load(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;
	int32_t pos;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);
	buf_ptr = osc_get_int32(buf_ptr, &pos);

	range_load(pos);
	size = CONFIG_SUCCESS("is", uuid, path);
	CONFIG_SEND(size);

	return 1;
}

static uint_fast8_t
_calibration_reset(const char *path, const char *fmt, uint_fast8_t argc, osc_data_t *buf)
{
	osc_data_t *buf_ptr = buf;
	uint16_t size;
	int32_t uuid;

	buf_ptr = osc_get_int32(buf_ptr, &uuid);

	// reset calibration range
	range_reset();

	size = CONFIG_SUCCESS("is", uuid, path);
	CONFIG_SEND(size);

	return 1;
}

/*
 * Query
 */

static const OSC_Query_Argument calibration_load_args [] = {
	OSC_QUERY_ARGUMENT_INT32("Slot", OSC_QUERY_MODE_W, 0, EEPROM_RANGE_MAX, 1)
};

static const OSC_Query_Argument calibration_save_args [] = {
	OSC_QUERY_ARGUMENT_INT32("Slot", OSC_QUERY_MODE_W, 0, EEPROM_RANGE_MAX, 1)
};

static const OSC_Query_Argument calibration_mid_args [] = {
	OSC_QUERY_ARGUMENT_FLOAT("Relative vicinity", OSC_QUERY_MODE_W, 0.f, 1.f, 0.01)
};

const OSC_Query_Item calibration_tree [] = {
	OSC_QUERY_ITEM_METHOD("load", "Load calibration from EEPROM", _calibration_load, calibration_load_args),
	OSC_QUERY_ITEM_METHOD("save", "Save calibration to EEPROM", _calibration_save, calibration_save_args),
	OSC_QUERY_ITEM_METHOD("reset", "Reset calibration to factory settings", _calibration_reset, NULL),

	OSC_QUERY_ITEM_METHOD("start", "Start calibration procedure", _calibration_start, NULL),
	OSC_QUERY_ITEM_METHOD("zero", "Calibrate quiescent values", _calibration_zero, NULL),
	OSC_QUERY_ITEM_METHOD("min", "Calibrate threshold values / curve fit point 1", _calibration_min, NULL),
	OSC_QUERY_ITEM_METHOD("mid", "Curve fit points 2-4", _calibration_mid, calibration_mid_args),
	OSC_QUERY_ITEM_METHOD("max", "Curve fit point 5", _calibration_max, NULL)
};
