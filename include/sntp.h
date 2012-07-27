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

#ifndef _SNTP_H_
#define _SNTP_H_

#include <stdint.h>

#include <netdef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t sntp_request (uint8_t *buf, timestamp64u_t now);
timestamp64u_t *sntp_dispatch (uint8_t *buf, timestamp64u_t now, timestamp64u_t *roundtrip_delay, timestamp64s_t *clock_offset);

#ifdef __cplusplus
}
#endif

#endif