/*
 * Copyright (c) 2012 Hanspeter Portner (agenthp@users.sf.net)
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

#include "config_private.h"
#include "../cmc/cmc_private.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include <libmaple/dsp.h>

#include <chimaera.h>
#include <chimutil.h>
#include <wiz.h>
#include <eeprom.h>
#include <cmc.h>
#include <scsynth.h>
#include <midi.h>

const char *success_str = "/success";
const char *fail_str = "/fail";
const char *wrong_ip_port_error_str = "wrong range: all numbers in IP must be < 0x100";
const char *group_err_str = "group not found";

#define CONFIG_SUCCESS(...) (nosc_message_vararg_serialize (&buf_o[buf_o_ptr][WIZ_SEND_OFFSET], success_str, __VA_ARGS__))
#define CONFIG_FAIL(...) (nosc_message_vararg_serialize (&buf_o[buf_o_ptr][WIZ_SEND_OFFSET], fail_str, __VA_ARGS__))

//FIXME solve this elegantly
//#define LAN_BROADCAST {255, 255, 255, 255} // global
//#define LAN_BROADCAST {169, 254, 255, 255} // IPv4LL
#define LAN_BROADCAST {192, 168, 1, 255} // local

//#define LAN_HOST {169, 254, 9, 90} // IPv4LL
#define LAN_HOST {192, 168, 1, 10} // local

float Y1 = 0.7;

Range range __CCM__;

float
_as (uint16_t qui, uint16_t out_s, uint16_t out_n, uint16_t b)
{
	float _qui = (float)qui;
	float _out_s = (float)out_s;
	float _out_n = (float)out_n;
	float _b = (float)b;

	return _qui / _b * (_out_s - _out_n) / (_out_s + _out_n);
}

Config config = {
	.magic = MAGIC, // used to compare EEPROM and FLASH config versions

	.name = {'c', 'h', 'i', 'm', 'a', 'e', 'r', 'a', '\0'},

	.version = {
		.major = 0,
		.minor = 1,
		.patch_level = 0
	},

	.comm = {
		.locally = 0,
		.mac = {(0x1a | 0b00000010) & 0b11111110, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f}, // locally administered unicast MAC
		.ip = {192, 168, 1, 177},
		.gateway = {192, 168, 1, 0},
		.subnet = {255, 255, 255, 0},
		.subnet_check = 0 //TODO make this configurable
	},

	.tuio = {
		.enabled = 0,
		.version = 2, //TODO implement
		.long_header = 0
	},

	.dump = {
		.enabled = 0
	},

	.scsynth = {
		.enabled = 0
	},

	.rtpmidi = {
		.enabled = 0,
	},

	.oscmidi = {
		.enabled = 0,
		.offset = 24,
		.effect = VOLUME
	},

	.dummy = {
		.enabled = 0,
	},
	
	.output = {
		.socket = {
			.sock = 1,
			.enabled = 1,
			.cb = output_enable,
			.port = {3333, 3333},
			.ip = LAN_BROADCAST
		},
		.offset = 0.001ULLK // := 1ms offset
	},

	.config = {
		.rate = 10, // rate in Hz
		.socket = {
			.sock = 2,
			.enabled = 1,
			.cb = config_enable,
			.port = {4444, 4444},
			.ip = LAN_BROADCAST
		}
	},

	.sntp = {
		.tau = 4, // delay between SNTP requests in seconds
		.socket = {
			.sock = 3,
			.enabled = 1,
			.cb = sntp_enable,
			.port = {123, 123},
			.ip = LAN_HOST
		}
	},

	.debug = {
		.socket = {
			.sock = 4,
			.enabled = 1,
			.cb = debug_enable,
			.port = {6666, 6666},
			.ip = LAN_BROADCAST
		}
	},

	.ipv4ll = {
		.enabled = 0
	},

	.mdns = {
		.socket = {
			.sock = 5,
			.enabled = 1,
			.cb = mdns_enable,
			.port = {5353, 5353}, // mDNS multicast port
			.ip = {224, 0, 0, 251} // mDNS multicast group
		}
	},

	.dhcpc = {
		.socket = {
			.sock = 6,
			.enabled = 0,
			.cb = dhcpc_enable,
			.port = {68, 67}, // BOOTPclient, BOOTPserver
			.ip = {255, 255, 255, 255} // broadcast
		}
	},

	.curve = {
		.A = 0.7700,
		.B = 0.2289,
		.C = 0.0000
	},

	.movingaverage = {
		.enabled = 1,
		.bitshift = 3 // moving average over 8(2³) samples
	},

	.interpolation = {
		//.order = 0, // use no interpolation at all
		//.order = 1, // use linear interpolation
		.order = 2, // use quadratic, aka hyperbolic interpolation
		//.order = 3, // use cubic interpolation
	},

	.rate = 2000, // update rate in Hz
	.pacemaker = 0x0b, // pacemaker rate 2^11=2048
	.calibration = 0, // use slot 0 as standard calibration
};

static uint8_t
magic_match ()
{
	uint8_t magic;
	eeprom_byte_read (eeprom_24LC64, EEPROM_CONFIG_OFFSET, &magic);

	return magic == config.magic; // check whether EEPROM and FLASH config magic number match
}

uint8_t
config_load ()
{
	if (magic_match ())
		eeprom_bulk_read (eeprom_24LC64, EEPROM_CONFIG_OFFSET, (uint8_t *)&config, sizeof (config));
	else // EEPROM and FLASH config version do not match, overwrite old with new default one
		config_save ();

	return 1;
}

uint8_t
config_save ()
{
	eeprom_bulk_write (eeprom_24LC64, EEPROM_CONFIG_OFFSET, (uint8_t *)&config, sizeof (config));
	return 1;
}

void
adc_fill (int16_t *raw12, int16_t *raw3, uint8_t *order12, uint8_t *order3, int16_t *sum, int16_t *rela, int16_t *swap)
{
	uint_fast8_t i;
	uint_fast8_t pos;
	uint16_t *qui = range.qui;
	uint32_t *rela_vec32 = (uint32_t *)rela;
	uint32_t *sum_vec32 = (uint32_t *)sum;
	uint32_t *qui_vec32 = (uint32_t *)range.qui;
	uint32_t *swap_vec32 = (uint32_t *)swap;

	uint32_t zero = 0UL;

	if (config.movingaverage.enabled)
		for (i=0; i<SENSOR_N/2; i++)
		{
			uint32_t mean = __shadd16 (sum_vec32[i], zero); // mean = sum / 2
			mean = __shadd16 (mean, zero); // mean /= 2
			mean = __shadd16 (mean, zero); // mean /= 2
			sum_vec32[i] = __ssub16 (sum_vec32[i], mean); // sum -= mean
		}

	for (i=0; i<MUX_MAX*ADC_DUAL_LENGTH*2; i++)
	{
		pos = order12[i];
		rela[pos] = raw12[i];
	}

	for (i=0; i<MUX_MAX*ADC_SING_LENGTH; i++)
	{
		pos = order3[i];
		rela[pos] = raw3[i];
	}

	if (config.movingaverage.enabled)
	{
		if (config.dump.enabled)
			for (i=0; i<SENSOR_N/2; i++)
			{
				uint32_t rela;
				rela = __ssub16 (rela_vec32[i], qui_vec32[i]); // SIMD sub

				sum_vec32[i] = __sadd16 (sum_vec32[i], rela); // sum += rela
				rela = __shadd16 (sum_vec32[i], zero); // rela = sum /2
				rela = __shadd16 (rela, zero); // rela /= 2
				rela = __shadd16 (rela, zero); // rela /= 2
				rela_vec32[i] = rela;

				swap_vec32[i] = __rev16 (rela); // SIMD hton
			}
		else // !config.dump.enabled
			for (i=0; i<SENSOR_N/2; i++)
			{
				uint32_t rela;
				rela = __ssub16 (rela_vec32[i], qui_vec32[i]); // SIMD sub

				sum_vec32[i] = __sadd16 (sum_vec32[i], rela); // sum += rela
				rela = __shadd16 (sum_vec32[i], zero); // rela = sum /2
				rela = __shadd16 (rela, zero); // rela /= 2
				rela = __shadd16 (rela, zero); // rela /= 2
				rela_vec32[i] = rela;
			}
	}
	else // !config.movingaverage.enabled
	{
		if (config.dump.enabled)
			for (i=0; i<SENSOR_N/2; i++)
			{
				rela_vec32[i] = __ssub16 (rela_vec32[i], qui_vec32[i]); // SIMD sub
				swap_vec32[i] = __rev16 (rela_vec32[i]); // SIMD hton
			}
		else // !config.dump.enabled
			for (i=0; i<SENSOR_N/2; i++)
			{
				rela_vec32[i] = __ssub16 (rela_vec32[i], qui_vec32[i]); // SIMD sub
			}
	}
}

uint8_t
range_load (uint8_t pos)
{
	if (magic_match ()) // EEPROM and FLASH config versions match
		eeprom_bulk_read (eeprom_24LC64, EEPROM_RANGE_OFFSET + pos*EEPROM_RANGE_SIZE, (uint8_t *)&range, sizeof (range));
	else // EEPROM and FLASH config version do not match, overwrite old with new default one
	{
		uint8_t i;
		for (i=0; i<SENSOR_N; i++)
		{
			range.thresh[i] = 0;
			range.qui[i] = 0x7ff;
			range.as_1_sc_1[i] = 1.0;
			range.bmin_sc_1 = 0.0;
		}

		range_save (pos);
	}

	return 1;
}

uint8_t
range_save (uint8_t pos)
{
	eeprom_bulk_write (eeprom_24LC64, EEPROM_RANGE_OFFSET + pos*EEPROM_RANGE_SIZE, (uint8_t *)&range, sizeof (range));

	return 1;
}

/*
nOSC_Arg _p [1];
nOSC_Arg _i [1];
nOSC_Arg _m [1];

nOSC_Arg _sa [1];
nOSC_Arg _sb [1];
nOSC_Arg _sc [1];
nOSC_Arg _st [1];

nOSC_Arg _na [1];
nOSC_Arg _nb [1];
nOSC_Arg _nc [1];
nOSC_Arg _nt [1];

const nOSC_Item _s [] = {
	nosc_message (_sa, "/A", "f"),
	nosc_message (_sb, "/B", "f"),
	nosc_message (_sc, "/C", "f"),
	nosc_message (_st, "/thresh", "i")
};

const nOSC_Item _n [] = {
	nosc_message (_na, "/A", "f"),
	nosc_message (_nb, "/B", "f"),
	nosc_message (_nc, "/C", "f"),
	nosc_message (_nt, "/thresh", "i")
};

const nOSC_Item calib_out [] = {
	nosc_message (_i, "/i", "i"),
	nosc_message (_m, "/mean", "i"),
	nosc_bundle ((nOSC_Item *)_s, nOSC_IMMEDIATE, "MMMM"),
	nosc_bundle ((nOSC_Item *)_n, nOSC_IMMEDIATE, "MMMM")
};

const char *calib_fmt = "MMBB";

uint8_t
range_print ()
{
	uint8_t i;
	for (i=0; i<SENSOR_N; i++)
	{
		nosc_message_set_int32 (_i, 0, i);
		nosc_message_set_int32 (_m, 0, adc_range[i].mean);

		nosc_message_set_float (_sa, 0, adc_range[i].A[POLE_SOUTH].fix);
		nosc_message_set_float (_sb, 0, adc_range[i].A[POLE_SOUTH].fix);
		nosc_message_set_float (_sc, 0, adc_range[i].B[POLE_SOUTH].fix);
		nosc_message_set_int32 (_st, 0, adc_range[i].thresh[POLE_SOUTH]);

		nosc_message_set_float (_na, 0, adc_range[i].A[POLE_NORTH].fix);
		nosc_message_set_float (_nb, 0, adc_range[i].A[POLE_NORTH].fix);
		nosc_message_set_float (_nc, 0, adc_range[i].B[POLE_NORTH].fix);
		nosc_message_set_int32 (_nt, 0, adc_range[i].thresh[POLE_NORTH]);

		uint16_t size = nosc_bundle_serialize ((nOSC_Item *)calib_out, nOSC_IMMEDIATE, (char *)calib_fmt, &buf_o[buf_o_ptr][WIZ_SEND_OFFSET]);
		udp_send (config.config.socket.sock, buf_o_ptr, size);
	}

	return 1;
}
*/

uint16_t arr [2][SENSOR_N]; //FIXME reuse some other memory
uint8_t zeroing = 0;

void
range_calibrate (int16_t *raw12, int16_t *raw3, uint8_t *order12, uint8_t *order3, int16_t *sum, int16_t *rela)
{
	uint8_t i;
	uint8_t pos;

	// fill rela vector from raw vector
	for (i=0; i<MUX_MAX*ADC_DUAL_LENGTH*2; i++)
	{
		pos = order12[i];
		rela[pos] = raw12[i];
	}

	for (i=0; i<MUX_MAX*ADC_SING_LENGTH; i++)
	{
		pos = order3[i];
		rela[pos] = raw3[i];
	}

	// do the calibration
	for (i=0; i<SENSOR_N; i++)
	{
		uint16_t avg;

		if (zeroing)
		{
			// moving average over 16 samples
			range.qui[i] -= range.qui[i] >> 4;
			range.qui[i] += rela[i];
		}

		//TODO is this the best way to get a mean of min and max?
		if (rela[i] > (avg = arr[POLE_SOUTH][i] >> 4) )
		{
			arr[POLE_SOUTH][i] -= avg;
			arr[POLE_SOUTH][i] += rela[i];
		}

		if (rela[i] < (avg =arr[POLE_NORTH][i] >> 4) )
		{
			arr[POLE_NORTH][i] -= avg;
			arr[POLE_NORTH][i] += rela[i];
		}
	}
}

// calibrate quiescent current
void
range_update_quiescent ()
{
	uint8_t i;

	for (i=0; i<SENSOR_N; i++)
	{
		// final average over last 16 samples
		range.qui[i] >>= 4;

		// reset array to quiescent value
		arr[POLE_SOUTH][i] = range.qui[i] << 4;
		arr[POLE_NORTH][i] = range.qui[i] << 4;
	}
}

// calibrate threshold
void
range_update_b0 ()
{
	uint8_t i;
	uint16_t thresh_s, thresh_n;

	for (i=0; i<SENSOR_N; i++)
	{
		arr[POLE_SOUTH][i] >>= 4; // average over 16 samples
		arr[POLE_NORTH][i] >>= 4;

		thresh_s = arr[POLE_SOUTH][i] - range.qui[i];
		thresh_n = range.qui[i] - arr[POLE_NORTH][i];

		range.thresh[i] = (thresh_s + thresh_n) / 2;

		// reset thresh to quiescent value
		arr[POLE_SOUTH][i] = range.qui[i] << 4;
		arr[POLE_NORTH][i] = range.qui[i] << 4;
	}
}

// calibrate amplification and sensitivity
void
range_update_b1 ()
{
	//FIXME approximate Y1, so that MAX ( (0x7ff * as_1_sc1[i]) - bmin_sc_1) == 1
	//FIXME or calculate Y1' out of A, B, C
	uint8_t i;
	uint16_t b = (float)0x7ff * Y1;
	float as_1;
	float bmin, bmax_s, bmax_n;
	float sc_1;

	float m_bmin = 0;
	float m_bmax = 0;

	for (i=0; i<SENSOR_N; i++)
	{
		arr[POLE_SOUTH][i] >>= 4; // average over 16 samples
		arr[POLE_NORTH][i] >>= 4;

		as_1 = 1.0 / _as (range.qui[i], arr[POLE_SOUTH][i], arr[POLE_NORTH][i], b);

		bmin = (float)range.thresh[i] * as_1;
		bmax_s = ((float)arr[POLE_SOUTH][i] - (float)range.qui[i]) * as_1 / Y1;
		bmax_n = ((float)range.qui[i] - (float)arr[POLE_NORTH][i]) * as_1 / Y1;

		m_bmin += bmin;
		m_bmax += (bmax_s + bmax_n) / 2.0;
	}

	m_bmin /= (float)SENSOR_N;
	m_bmax /= (float)SENSOR_N;

	sc_1 = 1.0 / (m_bmax - m_bmin);
	range.bmin_sc_1 = m_bmin * sc_1;

	for (i=0; i<SENSOR_N; i++)
	{
		as_1 = 1.0 / _as (range.qui[i], arr[POLE_SOUTH][i], arr[POLE_NORTH][i], b);
		range.as_1_sc_1[i] = as_1 * sc_1;

		// reset thresh to quiescent value
		arr[POLE_SOUTH][i] = range.qui[i] << 4;
		arr[POLE_NORTH][i] = range.qui[i] << 4;
	}
}

uint8_t
groups_load ()
{
	uint16_t size;
	uint8_t *buf;

	if (magic_match ())
	{
		buf = cmc_group_buf_get (&size);
		eeprom_bulk_read (eeprom_24LC64, EEPROM_GROUP_OFFSET, buf, size);
	}
	else
		groups_save ();

	return 1;
}

uint8_t
groups_save ()
{
	uint16_t size;
	uint8_t *buf;

	buf = cmc_group_buf_get (&size);
	eeprom_bulk_write (eeprom_24LC64, EEPROM_GROUP_OFFSET, buf, size);

	return 1;
}

static uint8_t
_check_bool (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args, uint8_t *boolean)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("ii", id, *boolean ? 1 : 0);
	}
	else
	{
		switch (fmt[1])
		{
			case nOSC_INT32:
				*boolean = args[1].i ? 1 : 0;
				break;
			case nOSC_TRUE:
				*boolean = 1;
				break;
			case nOSC_FALSE:
				*boolean = 0;
				break;
		}
		size = CONFIG_SUCCESS ("i", id);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_check_range8 (uint8_t *val, uint8_t min, uint8_t max, const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("ii", id, *val);
	}
	else
	{
		uint8_t arg = args[1].i;
		if ( (arg >= min) && (arg <= max) )
		{
			*val = arg;
			size = CONFIG_SUCCESS ("i", id);
		}
		else
		{
			char buf[64];
			sprintf (buf, "value %i is out of range [%i, %i]", arg, min, max);
			size = CONFIG_FAIL ("is", id, buf);
		}
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_check_range16 (uint16_t *val, uint16_t min, uint16_t max, const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("ii", id, *val);
	}
	else
	{
		uint16_t arg = args[1].i;
		if ( (arg >= min) && (arg <= max) )
		{
			*val = arg;
			size = CONFIG_SUCCESS ("i", id);
		}
		else
		{
			char buf[64];
			sprintf (buf, "value %i is out of range [%i, %i]", arg, min, max);
			size = CONFIG_FAIL ("is", id, buf);
		}
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_check_rangefloat (float *val, float min, float max, const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("if", id, *val);
	}
	else
	{
		float arg = args[1].f;
		if ( (arg >= min) && (arg <= max) )
		{
			*val = arg;
			size = CONFIG_SUCCESS ("i", id);
		}
		else
		{
			char buf[64];
			sprintf (buf, "value %i is out of range [%i, %i]", arg, min, max);
			size = CONFIG_FAIL ("is", id, buf);
		}
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_version (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	char version[16]; // FIXME share string buffer space between config methods
	sprintf (version, "%i.%i.%i", config.version.major, config.version.minor, config.version.patch_level);
	size = CONFIG_SUCCESS ("is", id, version);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_name (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("is", id, config.name);
	}
	else
	{
		if (strlen (args[1].s) < NAME_LENGTH)
		{
			strcpy (config.name, args[1].s);

			size = CONFIG_SUCCESS ("i", id);
		}
		else
			size = CONFIG_FAIL ("is", id, "name is too long");
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_config_load (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (config_load ())
		size = CONFIG_SUCCESS ("i", id);
	else
		size = CONFIG_FAIL ("is", id, "loading of configuration from EEPROM failed");

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_config_save (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (config_save ())
		size = CONFIG_SUCCESS ("i", id);
	else
		size = CONFIG_FAIL ("is", id, "saving configuration to EEPROM failed");

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

//TODO move to chimutil
#define MAC_STR_LEN 18
#define IP_STR_LEN 16
#define IP_STR_CIDR_LEN 19
#define ADDR_STR_LEN 32

static uint8_t
str2mac (char *str, uint8_t *mac)
{
	return sscanf (str, "%02x:%02x:%02x:%02x:%02x:%02x",
		&mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6;
}

static void
mac2str (uint8_t *mac, char *str)
{
	sprintf (str, "%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static uint8_t
str2ip (char *str, uint8_t *ip)
{
	return sscanf (str, "%hhi.%hhi.%hhi.%hhi",
		&ip[0], &ip[1], &ip[2], &ip[3]) == 4;
}

static uint8_t
str2ipCIDR (char *str, uint8_t *ip, uint8_t *mask)
{
	return sscanf (str, "%hhi.%hhi.%hhi.%hhi/%hhi",
		&ip[0], &ip[1], &ip[2], &ip[3], mask) == 5;
}

static void
ip2str (uint8_t *ip, char *str)
{
	sprintf (str, "%i.%i.%i.%i",
		ip[0], ip[1], ip[2], ip[3]);
}

static void
ip2strCIDR (uint8_t *ip, uint8_t mask, char *str)
{
	sprintf (str, "%i.%i.%i.%i/%i",
		ip[0], ip[1], ip[2], ip[3], mask);
}

static uint8_t
str2addr (char *str, uint8_t *ip, uint16_t *port)
{
	return sscanf (str, "%hhi.%hhi.%hhi.%hhi:%hi",
		&ip[0], &ip[1], &ip[2], &ip[3], port) == 5;
}

static void
addr2str (uint8_t *ip, uint16_t port, char *str)
{
	sprintf (str, "%i.%i.%i.%i:%i",
		ip[0], ip[1], ip[2], ip[3], port);
}

static uint8_t
_comm_locally (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _check_bool (path, fmt, argc, args, &config.comm.locally);
}

static uint8_t
_comm_mac (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		char mac_str[MAC_STR_LEN];
		mac2str (config.comm.mac, mac_str);
		size = CONFIG_SUCCESS ("is", id, mac_str);
	}
	else
	{
		uint8_t mac[6];
		if (str2mac (args[1].s, mac)) // TODO check if mac is valid
		{
			memcpy (config.comm.mac, mac, 6);
			wiz_mac_set (config.comm.mac);
			size = CONFIG_SUCCESS ("i", id);
		}
		else
			size = CONFIG_FAIL ("is", id, "wrong range: all numbers in MAC must be <0x100");
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_comm_ip (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	uint32_t *ip_ptr = (uint32_t *)config.comm.ip;
	uint32_t *subnet_ptr = (uint32_t *)config.comm.subnet;
	uint32_t *gateway_ptr = (uint32_t *)config.comm.gateway;

	if (argc == 1) // query
	{
		char ip_str_cidr[IP_STR_CIDR_LEN];
		uint8_t mask = subnet_to_cidr(config.comm.subnet);
		ip2strCIDR (config.comm.ip, mask, ip_str_cidr);
		size = CONFIG_SUCCESS ("is", id, ip_str_cidr);
	}
	else
	{
		uint8_t gtw[4];
		if(argc == 3)
		{
			if(str2ip(args[2].s, gtw))
			{
				memcpy(config.comm.gateway, gtw, 4);
				wiz_gateway_set(config.comm.gateway);
			}
			else // return
			{
				CONFIG_FAIL("is", id, "gatway invalid, format: x.x.x.x");
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				return 1;
			}
		}

		uint8_t ip[4];
		uint8_t mask;
		if (str2ipCIDR (args[1].s, ip, &mask))
		{
			memcpy (config.comm.ip, ip, 4);
			wiz_ip_set (config.comm.ip);

			cidr_to_subnet(config.comm.subnet, mask);
			wiz_subnet_set (config.comm.subnet);

			if(argc == 2) // no custom gateway was given
			{
				*gateway_ptr = (*ip_ptr) & (*subnet_ptr); // default gateway = (ip & subnet)
				wiz_gateway_set (config.comm.gateway);
			}
			size = CONFIG_SUCCESS ("i", id);
			//FIXME should we disable all sockets here?
		}
		else
			size = CONFIG_FAIL ("is", id, "ip invalid, format: x.x.x.x/x");
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_comm_gateway (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		char ip_str[IP_STR_LEN];
		ip2str (config.comm.gateway, ip_str);
		size = CONFIG_SUCCESS ("is", id, ip_str);
	}
	else
	{
		uint8_t ip[4];
		if (str2ip (args[1].s, ip)) //TODO check if valid
		{
			memcpy (config.comm.gateway, ip, 4);
			wiz_gateway_set (config.comm.gateway);
			size = CONFIG_SUCCESS ("i", id);
		}
		else
			size = CONFIG_FAIL ("is", id, wrong_ip_port_error_str);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_comm_subnet (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		char ip_str[IP_STR_LEN];
		ip2str (config.comm.subnet, ip_str);
		size = CONFIG_SUCCESS ("is", id, ip_str);
	}
	else
	{
		uint8_t ip[4];
		if (str2ip (args[1].s, ip)) //TODO check if valid
		{
			memcpy (config.comm.subnet, ip, 4);
			wiz_subnet_set (config.comm.subnet);
			size = CONFIG_SUCCESS ("i", id);
		}
		else
			size = CONFIG_FAIL ("is", id, wrong_ip_port_error_str);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_socket_enabled (Socket_Config *socket, const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
		size = CONFIG_SUCCESS ("ii", id, socket->enabled ? 1 : 0);
	else
	{
		socket->cb (args[1].i);
		size = CONFIG_SUCCESS ("i", id);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_output_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _socket_enabled (&config.output.socket, path, fmt, argc, args);
}

static uint8_t
_output_reset (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	config.dump.enabled = 0;
	config.tuio.enabled = 0;
	config.scsynth.enabled = 0;
	config.oscmidi.enabled = 0;
	config.dummy.enabled = 0;
	config.rtpmidi.enabled = 0;
	return 1;
}

static uint8_t
_config_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _socket_enabled (&config.config.socket, path, fmt, argc, args);
}

static uint8_t
_sntp_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _socket_enabled (&config.sntp.socket, path, fmt, argc, args);
}

static uint8_t
_debug_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _socket_enabled (&config.debug.socket, path, fmt, argc, args);
}

static uint8_t
_dhcpc_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _socket_enabled (&config.dhcpc.socket, path, fmt, argc, args);
}

const char *addr_err_str = "wrong range: port number must be < 0x10000 and numbers in IP must be < 0x100"; //TODO move me up

static void
_address_dns_cb (uint8_t *ip, void *data)
{
	uint16_t size;

	debug_str ("_address_dns_cb");
	Socket_Config *socket = data;

	memcpy (socket->ip, ip, 4);
	socket->cb (socket->enabled);

	size = CONFIG_SUCCESS ("iiii", ip[0], ip[1], ip[2], ip[3]);
	udp_send (config.config.socket.sock, buf_o_ptr, size);
}

static uint8_t
_address (Socket_Config *socket, const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	char *hostname = args[1].s;

	if (argc == 1) // query
	{
		char addr[ADDR_STR_LEN];
		addr2str (socket->ip, socket->port[DST_PORT], addr);
		size = CONFIG_SUCCESS ("is", id, addr);
	}
	else
	{
		uint16_t port;
		uint8_t ip[4];
		if (str2addr(hostname, ip, &port))
		{
			socket->port[DST_PORT] = port;
			_address_dns_cb(ip, socket);
			size = CONFIG_SUCCESS ("i", id);
		}
		else
		{
			if (strstr (hostname, ".local")) // TODO use const localdomain
			{
				char *port_str = strstr (hostname, ":");
				port = atoi (port_str+1);
				socket->port[DST_PORT] = port;

				mdns_resolve (hostname, _address_dns_cb, socket);
				size = CONFIG_SUCCESS ("i", id);
			}
			else
			{
				size = CONFIG_FAIL ("is", id, "can only resolve raw IP and mDNS addresses");
			}
		}
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_output_address (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _address (&config.output.socket, path, fmt, argc, args);
}

static uint8_t
_config_address (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _address (&config.config.socket, path, fmt, argc, args);
}

static uint8_t
_sntp_address (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _address (&config.sntp.socket, path, fmt, argc, args);
}

static uint8_t
_debug_address (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _address (&config.debug.socket, path, fmt, argc, args);
}

static void
_host_address_dns_cb (uint8_t *ip, void *data)
{
	uint16_t size;

	debug_str ("_host_address_dns_cb");

	size = CONFIG_SUCCESS ("iiii", ip[0], ip[1], ip[2], ip[3]);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	memcpy (config.output.socket.ip, ip, 4);
	memcpy (config.config.socket.ip, ip, 4);
	memcpy (config.sntp.socket.ip, ip, 4);
	memcpy (config.debug.socket.ip, ip, 4);

	output_enable (config.output.socket.enabled);
	config_enable (config.config.socket.enabled);
	sntp_enable (config.sntp.socket.enabled);
	debug_enable (config.debug.socket.enabled);
}

static uint8_t
_host_address (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	char *hostname = args[1].s;

	uint8_t ip[4];
	if (str2ip (hostname, ip)) // an IPv4 was given in string format
	{
		_host_address_dns_cb (ip, NULL);
		size = CONFIG_SUCCESS ("i", id);
	}
	else
	{
		if (strstr (hostname, ".local")) // resolve via mDNS
		{
			mdns_resolve (hostname, _host_address_dns_cb, NULL);
			size = CONFIG_SUCCESS ("i", id);
		}
		else // resolve via unicast DNS
		{
			size = CONFIG_FAIL ("is", "can only resolve raw IP and mDNS addresses");
			// FIXME TODO
		}
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_ipv4ll_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint8_t res = _check_bool (path, fmt, argc, args, &config.comm.locally);

	/*
	config.ipv4ll.enabled = bool;
	IPv4LL_claim (config.comm.ip, config.comm.gateway, config.comm.subnet);

	wiz_comm_set (config.comm.mac, config.comm.ip, config.comm.gateway, config.comm.subnet);
	*/


	return res;
}

static uint8_t
_config_rate (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _check_range16 (&config.config.rate, 1, 10, path, fmt, argc, args);
}

static uint8_t
_sntp_tau (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _check_range8 (&config.sntp.tau, 1, 10, path, fmt, argc, args);
}

static uint8_t
_tuio_long_header (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("ii", id, config.tuio.long_header ? 1 : 0);
	}
	else
	{
		tuio2_long_header_enable (args[1].i);
		size = CONFIG_SUCCESS ("i", id);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_tuio_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint8_t res = _check_bool (path, fmt, argc, args, &config.tuio.enabled);
	cmc_engines_update ();
	return res;
}

static uint8_t
_dump_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint8_t res = _check_bool (path, fmt, argc, args, &config.dump.enabled);
	cmc_engines_update ();
	return res;
}

static uint8_t
_scsynth_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint8_t res = _check_bool (path, fmt, argc, args, &config.scsynth.enabled);
	cmc_engines_update ();
	return res;
}

static uint8_t
_scsynth_group (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	uint16_t gid = args[1].i;

	char *name;
	uint16_t sid;
	uint16_t group;
	uint16_t out;
	uint8_t arg;
	uint8_t alloc;
	uint8_t gate;
	uint8_t add_action;
	uint8_t is_group;

	if (argc == 2) // query
	{
		scsynth_group_get(gid, &name, &sid, &group, &out, &arg, &alloc, &gate, &add_action, &is_group);

		size = CONFIG_SUCCESS ("isiiiiiiii", id, name, sid, group, out, arg, alloc, gate, add_action, is_group);
	}
	else
	{
		name = args[2].s;
		sid = args[3].i;
		group = args[4].i;
		out = args[5].i;
		arg = args[6].i;
		alloc = args[7].i;
		gate = args[8].i;
		add_action = args[9].i;
		is_group = args[10].i;

		scsynth_group_set(gid, name, sid, group, out, arg, alloc, gate, add_action, is_group);

		size = CONFIG_SUCCESS ("i", id);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_rtpmidi_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint8_t res = _check_bool (path, fmt, argc, args, &config.rtpmidi.enabled);
	cmc_engines_update ();
	return res;
}

static uint8_t
_oscmidi_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint8_t res = _check_bool (path, fmt, argc, args, &config.oscmidi.enabled);
	cmc_engines_update ();
	return res;
}

static uint8_t
_oscmidi_offset (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _check_range8 (&config.oscmidi.offset, 0, 0x7f, path, fmt, argc, args);
}

static uint8_t
_oscmidi_effect (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _check_range8 (&config.oscmidi.effect, 0, 0x7f, path, fmt, argc, args);
}

static uint8_t
_dummy_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint8_t res = _check_bool (path, fmt, argc, args, &config.dummy.enabled);
	cmc_engines_update ();
	return res;
}

static uint8_t
_output_offset (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("id", id, (double)config.output.offset); // output timestamp, double, float?
	}
	else
	{
		switch (fmt[1])
		{
			case nOSC_TIMESTAMP:
				config.output.offset = args[1].t;
				break;
			case nOSC_FLOAT:
				config.output.offset = args[1].f;
				break;
			case nOSC_DOUBLE:
				config.output.offset = args[1].d;
				break;
		}
		size = CONFIG_SUCCESS ("i", id);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_rate (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		if (config.rate > 0)
			size = CONFIG_SUCCESS ("ii", id, config.rate);
		else // infinity
			size = CONFIG_SUCCESS ("ii", id, nOSC_Infty);
	}
	else
	{
		if (args[1].i < nOSC_Infty) // TODO also check 16bit size
			config.rate = args[1].i;
		else
			config.rate = 0;

		if (config.rate)
		{
			timer_pause (adc_timer);
			adc_timer_reconfigure ();
			timer_resume (adc_timer);
		}
		else
			timer_pause (adc_timer);

		size = CONFIG_SUCCESS ("i", id);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_reset (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	int32_t sec;

	if (argc > 1)
	{
		sec = args[1].i;
		if (sec < 1)
			sec = 1;
	}
	else
		sec = 1;

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	delay_us (sec * 1e6); // delay sec seconds until reset
	nvic_sys_reset ();

	return 1;
}

static uint8_t
_factory (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	int32_t sec;

	if (argc > 1)
	{
		sec = args[1].i;
		if (sec < 1)
			sec = 1;
	}
	else
		sec = 1;

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	// FIXME does not work as intended without VBAT powered up, needs a change on the PCB
	//bkp_init ();
	//bkp_enable_writes ();
	//bkp_write (FACTORY_RESET_REG, FACTORY_RESET_VAL);
	//bkp_disable_writes ();

	delay_us (sec * 1e6); // delay sec seconds until reset
	nvic_sys_reset ();

	return 1;
}

static uint8_t
_curve (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("ifff", id, (float)config.curve.A, (float)config.curve.B, (float)config.curve.C);
	}
	else
	{
		config.curve.A = args[1].f;
		config.curve.B = args[2].f;
		config.curve.B = args[3].f;

		size = CONFIG_SUCCESS ("i", id);
	}

	udp_send (config.config.socket.sock, buf_o_ptr, size);
}

static uint8_t
_movingaverage_enabled (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _check_bool (path, fmt, argc, args, &config.movingaverage.enabled);
}

static uint8_t
_movingaverage_samples (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (argc == 1) // query
	{
		size = CONFIG_SUCCESS ("ii", id, 1U << config.movingaverage.bitshift);
	}
	else
		switch (args[1].i)
		{
			case 2:
				config.movingaverage.bitshift = 1;
				size = CONFIG_SUCCESS ("i", id);
				break;
			case 4:
				config.movingaverage.bitshift = 2;
				size = CONFIG_SUCCESS ("i", id);
				break;
			case 8:
				config.movingaverage.bitshift = 3;
				size = CONFIG_SUCCESS ("i", id);
				break;
			case 16:
				config.movingaverage.bitshift = 4;
				size = CONFIG_SUCCESS ("i", id);
				break;
			default:
				size = CONFIG_FAIL ("is", id, "valid sample windows are 2, 4, 8 and 16");
		}

	udp_send (config.config.socket.sock, buf_o_ptr, size);
}

static uint8_t
_interpolation_order (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	return _check_range8 (&config.interpolation.order, 0, 3, path, fmt, argc, args);
}

static uint8_t
_group_clear (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	cmc_group_clear ();

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_group_get (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	uint16_t pid;
	float x0;
	float x1;

	if (cmc_group_get (args[1].i, &pid, &x0, &x1))
		size = CONFIG_SUCCESS ("iiff", id, pid, x0, x1);
	else
		size = CONFIG_FAIL ("is", id, group_err_str);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_group_set (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (cmc_group_set (args[1].i, args[2].i, args[3].f, args[4].f))
		size = CONFIG_SUCCESS ("i", id);
	else
		size = CONFIG_FAIL ("is", id, group_err_str);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_group_load (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (groups_load ())
		size = CONFIG_SUCCESS ("i", id);
	else
		size = CONFIG_FAIL ("is", id, "groups could not be loaded from EEPROM");
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_group_save (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	if (groups_save ())
		size = CONFIG_SUCCESS ("i", id);
	else
		size = CONFIG_FAIL ("is", id, "groups could not be saved to EEPROM");
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_calibration_start (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	// initialize sensor range
	uint8_t i;
	for (i=0; i<SENSOR_N; i++)
	{
		// moving average over 16 samples
		range.qui[i] = ADC_HALF_BITDEPTH << 4;
		
		arr[POLE_SOUTH][i] = range.qui[i];
		arr[POLE_NORTH][i] = range.qui[i];
	}

	// enable calibration
	zeroing = 1;
	calibrating = 1;

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_calibration_zero (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	// update new range
	zeroing = 0;
	range_update_quiescent ();

	uint8_t i;
	for (i=0; i<SENSOR_N; i++)
	{
		size = nosc_message_vararg_serialize (&buf_o[buf_o_ptr][WIZ_SEND_OFFSET], "/range/qui", "iii", i, range.qui[i], range.qui[i]-0x7ff);
		udp_send (config.config.socket.sock, buf_o_ptr, size);
	}

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_calibration_min (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	// update new range
	range_update_b0 ();

	uint8_t i;
	for (i=0; i<SENSOR_N; i++)
	{
		size = nosc_message_vararg_serialize (&buf_o[buf_o_ptr][WIZ_SEND_OFFSET], "/range/thresh", "ii", i, range.thresh[i]);
		udp_send (config.config.socket.sock, buf_o_ptr, size);
	}

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_calibration_mid (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	Y1 = args[1].f;

	// update new range
	range_update_b1 ();

	uint8_t i;
	for (i=0; i<SENSOR_N; i++)
	{
		size = nosc_message_vararg_serialize (&buf_o[buf_o_ptr][WIZ_SEND_OFFSET], "/range/sc_1_sc_1", "if", i, (float)range.as_1_sc_1[i]);
		udp_send (config.config.socket.sock, buf_o_ptr, size);
	}

	calibrating = 0;

	size = nosc_message_vararg_serialize (&buf_o[buf_o_ptr][WIZ_SEND_OFFSET], "/range/bmin_sc_1", "f", (float)range.bmin_sc_1);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_calibration_save (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	uint8_t pos = config.calibration; // use default calibration

	if (argc == 2)
		pos = args[1].i; // use given calibration

	if (pos > EEPROM_RANGE_MAX)
		pos = EEPROM_RANGE_MAX;

	// store new calibration range to EEPROM
	range_save (pos);

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_calibration_load (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	uint8_t pos = config.calibration; // use default calibration

	if (argc == 2)
		pos = args[1].i; // use given calibration

	if (pos > EEPROM_RANGE_MAX)
		pos = EEPROM_RANGE_MAX;

	// load calibration range from EEPROM
	range_load (pos);

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_calibration_print (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	// print calibration in RAM
	//range_print (); FIXME

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_uid (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	size = CONFIG_SUCCESS ("is", id, EUI_96_STR);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static void
_resolve_done (uint8_t *ip, void *data)
{
	uint16_t size;

	size = CONFIG_SUCCESS ("iiii", ip[0], ip[1], ip[2], ip[3]);
	udp_send (config.config.socket.sock, buf_o_ptr, size);
}

static uint8_t
_resolve (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;
	const char *name = args[1].s;

	mdns_resolve (name, _resolve_done, NULL);

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

/*
static uint8_t
_test (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	debug_int32 (sizeof (sat unsigned short fract));				// size = 1
	debug_int32 (sizeof (sat unsigned fract));							// size = 2
	debug_int32 (sizeof (sat unsigned long fract));					// size = 4
	debug_int32 (sizeof (sat unsigned long long fract));		// size = 8

	debug_int32 (sizeof (sat short fract));									// size = 1
	debug_int32 (sizeof (sat fract));												// size = 2
	debug_int32 (sizeof (sat long fract));									// size = 4
	debug_int32 (sizeof (sat long long fract));							// size = 8

	debug_int32 (sizeof (sat unsigned short accum));				// size = 2
	debug_int32 (sizeof (sat unsigned accum));							// size = 4
	debug_int32 (sizeof (sat unsigned long accum));					// size = 8
	debug_int32 (sizeof (sat unsigned long long accum));		// size = 8

	debug_int32 (sizeof (sat short accum));									// size = 2
	debug_int32 (sizeof (sat accum));												// size = 4
	debug_int32 (sizeof (sat long accum));									// size = 8
	debug_int32 (sizeof (sat long long accum));							// size = 8

	debug_str ("nOSC_Arg");
	debug_int32 (sizeof (nOSC_Type));
	debug_int32 (sizeof (nOSC_Arg));
	debug_int32 (sizeof (nOSC_Item));
	debug_int32 (sizeof (nOSC_Method));
	debug_int32 (sizeof (nOSC_Timestamp));
	debug_int32 (sizeof (int32_t));
	debug_int32 (sizeof (float));
	debug_int32 (sizeof (char *));
	debug_int32 (sizeof (nOSC_Blob));
	debug_int32 (sizeof (int64_t));
	debug_int32 (sizeof (double));
	debug_int32 (sizeof (uint8_t [4]));
	debug_int32 (sizeof (char *));
	debug_int32 (sizeof (char));

	struct {
		union {
			uint32_t i;
		} u;
		char c;
	} s;
	debug_int32 (sizeof (s));

	size = CONFIG_SUCCESS ("i", id);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

static uint8_t
_echo (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	uint8_t i;
	for (i=1; i<argc; i++)
		switch (fmt[i])
		{
			case nOSC_INT32:
				CONFIG_SUCCESS ("ii", id, args[i].i);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_FLOAT:
				CONFIG_SUCCESS ("if", id, args[i].f);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_STRING:
				CONFIG_SUCCESS ("is", id, args[i].s);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_BLOB:
				CONFIG_SUCCESS ("ib", id, args[i].b.size, args[i].b.data);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;

			case nOSC_TRUE:
				CONFIG_SUCCESS ("iT", id);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_FALSE:
				CONFIG_SUCCESS ("iF", id);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_NIL:
				CONFIG_SUCCESS ("iN", id);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_INFTY:
				CONFIG_SUCCESS ("iI", id);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;

			case nOSC_INT64:
				CONFIG_SUCCESS ("ih", id, args[i].h);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_DOUBLE:
				CONFIG_SUCCESS ("id", id, args[i].d);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_TIMESTAMP:
				CONFIG_SUCCESS ("it", id, args[i].t);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;

			case nOSC_MIDI:
				CONFIG_SUCCESS ("im", id, args[i].m);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_SYMBOL:
				CONFIG_SUCCESS ("iS", id, args[i].S);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
			case nOSC_CHAR:
				CONFIG_SUCCESS ("ic", id, args[i].c);
				udp_send (config.config.socket.sock, buf_o_ptr, size);
				break;
		}

	return 1;
}
*/

static uint8_t
_non (const char *path, const char *fmt, uint8_t argc, nOSC_Arg *args)
{
	uint16_t size;
	int32_t id = args[0].i;

	size = CONFIG_FAIL ("isss", id, "unknown method for path or format", path, fmt);
	udp_send (config.config.socket.sock, buf_o_ptr, size);

	return 1;
}

const nOSC_Method config_serv [] = {
	{"/chimaera/version", "i", _version},

	{"/chimaera/name", "i*", _name},

	{"/chimaera/config/load", "i", _config_load},
	{"/chimaera/config/save", "i", _config_save},

	{"/chimaera/comm/locally", "i*", _comm_locally},
	{"/chimaera/comm/mac", "i*", _comm_mac},
	{"/chimaera/comm/ip", "i*", _comm_ip},
	//{"/chimaera/comm/gateway", "i*", _comm_gateway},
	//{"/chimaera/comm/subnet", "i*", _comm_subnet},

	{"/chimaera/output/enabled", "i*", _output_enabled},
	{"/chimaera/output/address", "i*", _output_address},
	{"/chimaera/output/offset", "i*", _output_offset},
	{"/chimaera/output/reset", "i", _output_reset},

	{"/chimaera/dump/enabled", "i*", _dump_enabled},

	{"/chimaera/tuio/enabled", "i*", _tuio_enabled},
	{"/chimaera/tuio/long_header", "i*", _tuio_long_header},

	{"/chimaera/scsynth/enabled", "i*", _scsynth_enabled},
	{"/chimaera/scsynth/group", "ii*", _scsynth_group},

	{"/chimaera/oscmidi/enabled", "i*", _oscmidi_enabled},
	{"/chimaera/oscmidi/offset", "i*", _oscmidi_offset},
	{"/chimaera/oscmidi/effect", "i*", _oscmidi_effect},

	{"/chimaera/dummy/enabled", "i*", _dummy_enabled},

	{"/chimaera/rtpmidi/enabled", "i*", _rtpmidi_enabled},

	{"/chimaera/config/enabled", "i*", _config_enabled},
	{"/chimaera/config/address", "i*", _config_address},
	{"/chimaera/config/rate", "i*", _config_rate},

	{"/chimaera/sntp/enabled", "i*", _sntp_enabled},
	{"/chimaera/sntp/address", "i*", _sntp_address},
	{"/chimaera/sntp/tau", "i*", _sntp_tau},

	{"/chimaera/debug/enabled", "i*", _debug_enabled},
	{"/chimaera/debug/address", "i*", _debug_address},

	{"/chimaera/dhcpc/enabled", "i*", _dhcpc_enabled},

	{"/chimaera/host/address", "is", _host_address},

	//TODO
	//{"/chimaera/mdns/enabled", "i*", _mdns_enabled},

	{"/chimaera/ipv4ll/enabled", "i*", _ipv4ll_enabled},

	{"/chimaera/curve", "i", _curve},
	{"/chimaera/curve", "ifff", _curve},

	{"/chimaera/movingaverage/enabled", "i*", _movingaverage_enabled},
	{"/chimaera/movingaverage/samples", "i*", _movingaverage_samples},

	{"/chimaera/interpolation/order", "i*", _interpolation_order},

	{"/chimaera/group/clear", "i", _group_clear},
	{"/chimaera/group/get", "ii", _group_get},
	{"/chimaera/group/set", "iiiff", _group_set},

	{"/chimaera/group/load", "i", _group_load},
	{"/chimaera/group/save", "i", _group_save},

	{"/chimaera/rate", "i*", _rate},

	{"/chimaera/reset", "i*", _reset},

	{"/chimaera/factory", "i*", _factory},

	{"/chimaera/calibration/start", "i", _calibration_start},
	{"/chimaera/calibration/zero", "i", _calibration_zero},
	{"/chimaera/calibration/min", "i", _calibration_min},
	{"/chimaera/calibration/mid", "if", _calibration_mid},
	{"/chimaera/calibration/print", "i", _calibration_print},
	{"/chimaera/calibration/save", "i*", _calibration_save},
	{"/chimaera/calibration/load", "i*", _calibration_load},

	{"/chimaera/uid", "i", _uid},

	{"/chimaera/resolve", "is", _resolve},

	//TODO remove
	/*
	{"/chimaera/test", "i", _test},
	{"/chimaera/echo", NULL, _echo},
	*/

	{NULL, NULL, _non}, // if nothing else matches, we give back an error saying so

	{NULL, NULL, NULL} // terminator
};
