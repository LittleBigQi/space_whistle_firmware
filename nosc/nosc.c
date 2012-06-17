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

#include <string.h>

#include "nosc_private.h"

//TODO get rid of calloc and strdup and use configurable buffers for it

/*
 * Server
 */

nOSC_Server * 
nosc_server_method_add (nOSC_Server *serv, const char *path, const char *fmt, nOSC_Method_Cb cb, void *data)
{
	nOSC_Server *new = calloc (1, sizeof (nOSC_Server));
	if (path)
		new->path = strdup (path);
	if (fmt)
		new->fmt = strdup (fmt);
	new->cb = cb;
	new->data = data;
	new->next = serv;
	return new;
}

void
_nosc_server_message_dispatch (nOSC_Server *serv, nOSC_Message *msg, char *path, char *fmt)
{
	nOSC_Server *ptr = serv;
	while (ptr)
	{
		// raw matches only of path and format strings
		if ( ptr->path && !strcmp (ptr->path, path))
			//if ( ptr->fmt && !strcmp (ptr->fmt, fmt)) //TODO not working properly atm
			{
				nOSC_Message *tmp = msg;

				uint8_t argc = 1;
				while (tmp && tmp->prev)
				{
					tmp = tmp->prev;
					argc++;
				}

				nOSC_Arg **argv = calloc (argc, sizeof (nOSC_Arg *));

				uint8_t i = 0;
				while (tmp)
				{
					argv[i++] = &tmp->arg;
					tmp = tmp->next;
				}

				uint8_t res = ptr->cb (ptr->data, path, fmt, argc, argv);

				free (argv);

				if (res) // return when handled
					return;
			}

		ptr = ptr->next;
	}
}

void
nosc_server_dispatch (nOSC_Server *serv, uint8_t *buf, uint16_t size)
{
	nOSC_Bundle *bund;
	nOSC_Message *msg;
	char *path;
	char *fmt;

	if (buf[0] == '#') // check whether we are a bundle
	{
		bund = _nosc_bundle_deserialize (buf, size);
		nOSC_Bundle *ptr = bund;

		while (ptr)
		{
			_nosc_server_message_dispatch (serv, ptr->msg, ptr->path, ptr->fmt);
			ptr = ptr->prev;
		}

		nosc_bundle_free (bund);
	}
	else if (buf[0] == '/') // check whether we are a message
	{
		msg = _nosc_message_deserialize (buf, size, &path, &fmt);
		_nosc_server_message_dispatch (serv, msg, path, fmt);
		nosc_message_free (msg);
	}
}

void 
nosc_server_free (nOSC_Server *serv)
{
	nOSC_Server *ptr = serv;
	while (ptr)
	{
		nOSC_Server *tmp = ptr;
		ptr = ptr->next;
		if (tmp->path)
			free (tmp->path);
		if (tmp->fmt)
			free (tmp->fmt);
		free (tmp);
	}
}

nOSC_Bundle *
_nosc_bundle_deserialize (uint8_t *buf, uint16_t size)
{
	int32_t msg_size;
	nOSC_Bundle *bund = NULL;
	nOSC_Message *msg;
	char *path;
	char *fmt;

	uint8_t *buf_ptr = buf;

	buf_ptr += 8; // skip "#bundle"
	buf_ptr += 8; // timestamp is ignored, we have too little memory to store messages for later dispatching

	while (buf_ptr-buf < size)
	{
		msg_size = (int32_t) *buf_ptr;
		buf_ptr += 4;

		msg = _nosc_message_deserialize (buf_ptr, msg_size, &path, &fmt);
		buf_ptr += msg_size;
		bund = nosc_bundle_add_message (bund, msg, path);
	}

	return bund;
}

nOSC_Message *
_nosc_message_deserialize (uint8_t *buf, uint16_t size, char **path, char **fmt)
{
	nOSC_Message *msg = NULL;

	uint8_t *buf_ptr = buf;
	uint8_t len;

	// find path
	*path = buf_ptr;
	len = strlen (*path)+1;
	if (len%4)
		len += 4 - len%4;
	buf_ptr += len;

	// find format
	*fmt = buf_ptr;
	len = strlen (*fmt)+1;
	if (len%4)
		len += 4 - len%4;
	buf_ptr += len;
	
	(*fmt)++; // skip ','

	uint8_t *type = *fmt;
	while (*type != '\0')
	{
		switch (*type)
		{
			case nOSC_INT32:
			{
				int32_t *i = (int32_t *) buf_ptr;
				msg = nosc_message_add_int32 (msg, *i);
				msg->arg.i = htonl (msg->arg.i);
				buf_ptr += 4;
				break;
			}
			case nOSC_FLOAT:
			{
				float *f = (float *) buf_ptr;
				msg = nosc_message_add_float (msg, *f);
				msg->arg.i = htonl (msg->arg.i);
				buf_ptr += 4;
				break;
			}
			case nOSC_STRING:
			{
				char *s = (char *) buf_ptr;
				msg = nosc_message_add_string (msg, s);
				len = strlen (s)+1;
				if (len%4)
					len += 4 - len%4;
				buf_ptr += len;
				break;
			}

			case nOSC_TRUE:
				msg = nosc_message_add_true (msg);
				break;
			case nOSC_FALSE:
				msg = nosc_message_add_false (msg);
				break;
			case nOSC_NIL:
				msg = nosc_message_add_nil (msg);
				break;
			case nOSC_INFTY:
				msg = nosc_message_add_infty (msg);
				break;

			case nOSC_DOUBLE:
			{
				double *d = (double *) buf_ptr;
				msg = nosc_message_add_double (msg, *d);
				msg->arg.h = htonll (msg->arg.h);
				buf_ptr += 8;
				break;
			}
			case nOSC_INT64:
			{
				int64_t *h = (int64_t *) buf_ptr;
				msg = nosc_message_add_int64 (msg, *h);
				msg->arg.h = htonll (msg->arg.h);
				buf_ptr += 8;
				break;
			}
			case nOSC_TIMESTAMP:
			{
				uint64_t *tt = (uint64_t *) buf_ptr;
				nOSC_Timestamp t = {*tt};
				msg = nosc_message_add_timestamp (msg, t);
				msg->arg.h = htonll (msg->arg.h);
				buf_ptr += 8;
				break;
			}

			case nOSC_MIDI:
			{
				uint8_t *m = buf_ptr;
				msg = nosc_message_add_midi (msg, m);
				buf_ptr += 4;
				break;
			}
		}
		type++;
	}

	return msg;
}

/*
 * Bundle
 */

nOSC_Bundle *
nosc_bundle_add_message (nOSC_Bundle *bund, nOSC_Message *msg, const char *path)
{
	nOSC_Bundle *new = calloc (1, sizeof (nOSC_Bundle));
	new->msg = msg;
	new->path = strdup (path);
	new->prev = bund;
	if (bund) 
		bund->next = new;

	// create format string

	// get first element
	nOSC_Message *first = msg;
	while (first && first->prev)
		first = first->prev;

	// write format
	char fmt [32]; //TODO what size to use?
	char *fmt_ptr = fmt;
	nOSC_Message *ptr = first;
	while (ptr)
	{
		*fmt_ptr++ = ptr->type;
		ptr = ptr->next;
	}
	*fmt_ptr++ = '\0';
	new->fmt = strdup (fmt);

	return new;
}

uint16_t
nosc_bundle_serialize (nOSC_Bundle *bund, nOSC_Timestamp timestamp, uint8_t *buf)
{
	uint8_t *buf_ptr = buf;

	buf[0] = '#';
	buf[1] = 'b';
	buf[2] = 'u';
	buf[3] = 'n';
	buf[4] = 'd';
	buf[5] = 'l';
	buf[6] = 'e';
	buf[7] = '\0';
	buf_ptr += 8;

	uint64_t tt = htonll (timestamp.all);
	memcpy (buf_ptr, &tt, 8);
	buf_ptr += 8;

	// get first bundle
	nOSC_Bundle *first = bund;
	while (first && first->prev)
		first = first->prev;

	nOSC_Bundle *ptr = first;
	while (ptr)
	{
		uint16_t msg_size;
		int32_t i;

		msg_size = nosc_message_serialize (ptr->msg, ptr->path, buf_ptr+4);
		i = htonl (msg_size);
		memcpy (buf_ptr, &i, 4);
		buf_ptr += msg_size + 4;

		ptr = ptr->next;
	}

	return buf_ptr - buf;
}

void 
nosc_bundle_free (nOSC_Bundle *bund)
{
	nOSC_Bundle *ptr = bund;
	while (ptr)
	{
		nOSC_Bundle *tmp = ptr;
		ptr = ptr->prev;
		nosc_message_free (tmp->msg);
		if (tmp->path)
			free (tmp->path);
		if (tmp->fmt)
			free (tmp->fmt);
		free (tmp);
	}
}

/*
 * Message
 */

static nOSC_Message *
_msg_add (nOSC_Message *msg, nOSC_Type type)
{
	nOSC_Message *new = calloc (1, sizeof (nOSC_Message));
	new->type = type;
	new->prev = msg;
	if (msg) 
		msg->next = new;

	return new;
}

nOSC_Message *
nosc_message_add_int32 (nOSC_Message *msg, int32_t i)
{
	nOSC_Message *new = _msg_add (msg, nOSC_INT32);
	new->arg.i = i;
	return new;
}

nOSC_Message *
nosc_message_add_float (nOSC_Message *msg, float f)
{
	nOSC_Message *new = _msg_add (msg, nOSC_FLOAT);
	new->arg.f = f;
	return new;
}

nOSC_Message * 
nosc_message_add_string (nOSC_Message *msg, const char *s)
{
	nOSC_Message *new = _msg_add (msg, nOSC_STRING);
	new->arg.s = strdup (s);
	return new;
}

nOSC_Message * 
nosc_message_add_true (nOSC_Message *msg)
{
	nOSC_Message *new = _msg_add (msg, nOSC_TRUE);
	return new;
}

nOSC_Message * 
nosc_message_add_false (nOSC_Message *msg)
{
	nOSC_Message *new = _msg_add (msg, nOSC_FALSE);
	return new;
}

nOSC_Message * 
nosc_message_add_nil (nOSC_Message *msg)
{
	nOSC_Message *new = _msg_add (msg, nOSC_NIL);
	return new;
}

nOSC_Message * 
nosc_message_add_infty (nOSC_Message *msg)
{
	nOSC_Message *new = _msg_add (msg, nOSC_INFTY);
	return new;
}

nOSC_Message * 
nosc_message_add_double (nOSC_Message *msg, double d)
{
	nOSC_Message *new = _msg_add (msg, nOSC_DOUBLE);
	new->arg.d = d;
	return new;
}

nOSC_Message * 
nosc_message_add_int64 (nOSC_Message *msg, int64_t h)
{
	nOSC_Message *new = _msg_add (msg, nOSC_INT64);
	new->arg.h = h;
	return new;
}

nOSC_Message * 
nosc_message_add_timestamp (nOSC_Message *msg, nOSC_Timestamp t)
{
	nOSC_Message *new = _msg_add (msg, nOSC_TIMESTAMP);
	new->arg.t = t;
	return new;
}

nOSC_Message * 
nosc_message_add_midi (nOSC_Message *msg, uint8_t m [4])
{
	nOSC_Message *new = _msg_add (msg, nOSC_MIDI);
	memcpy (new->arg.m, m, 4);
	return new;
}

uint16_t
nosc_message_serialize (nOSC_Message *msg, const char *path, uint8_t *buf)
{
	uint8_t i;
	uint16_t len;

	// resetting buf_ptr to start of buf
	uint8_t *buf_ptr = buf;

	// get first element
	nOSC_Message *first = msg;
	while (first && first->prev)
		first = first->prev;

	// write path
	len = strlen (path) + 1;
	memcpy (buf_ptr, path, len);
	buf_ptr += len;
	if (len%4)
		for (i=len%4; i<4; i++)
			*buf_ptr++ = '\0';

	// write format
	nOSC_Message *ptr = first;
	*buf_ptr++ = ',';
	len = 1;
	while (ptr)
	{
		*buf_ptr++ = ptr->type;
		len++;
		ptr = ptr->next;
	}
	*buf_ptr++ = '\0';
	len++;
	if (len%4)
		for (i=len%4; i<4; i++)
			*buf_ptr++ = '\0';

	// write arguments
	ptr = first;
	while (ptr)
	{
		switch (ptr->type)
		{
			case nOSC_INT32:
			case nOSC_FLOAT:
			{
				int32_t i = htonl (ptr->arg.i);
				memcpy (buf_ptr, &i, 4);
				buf_ptr += 4;
				break;
			}

			case nOSC_MIDI:
			{
				memcpy (buf_ptr, ptr->arg.m, 4);
				buf_ptr += 4;
				break;
			}

			case nOSC_STRING:
			{
				uint8_t i;
				char *s = ptr->arg.s;
				uint16_t len = strlen (s) + 1;
				memcpy (buf_ptr, s, len);
				buf_ptr += len;
				if (len%4)
					for (i=len%4; i<4; i++)
						*buf_ptr++ = '\0';
				break;
			}

			case nOSC_TRUE:
			case nOSC_FALSE:
			case nOSC_NIL:
			case nOSC_INFTY:
				break;

			case nOSC_DOUBLE:
			case nOSC_INT64:
			case nOSC_TIMESTAMP:
			{
				int64_t h = htonll (ptr->arg.h);
				memcpy (buf_ptr, &h, 8);
				buf_ptr += 8;
				break;
			}
		}
		ptr = ptr->next;
	}

	return buf_ptr - buf;
}

uint16_t
nosc_message_vararg_serialize (uint8_t *buf, const char *path, const char *fmt, ...)
{
	nOSC_Message *msg = NULL;

  va_list args;
  va_start (args, fmt);

  const char *p;
  for (p = fmt; *p != '\0'; p++)
		switch (*p)
		{
			case nOSC_INT32:
				msg = nosc_message_add_int32 (msg, va_arg (args, int32_t));
        break;
			case nOSC_FLOAT:
				msg = nosc_message_add_float (msg, (float) va_arg (args, double));
        break;
			case nOSC_STRING:
				msg = nosc_message_add_string (msg, va_arg (args, char *));
        break;

			case nOSC_TRUE:
				msg = nosc_message_add_true (msg);
        break;
			case nOSC_FALSE:
				msg = nosc_message_add_false (msg);
        break;
			case nOSC_NIL:
				msg = nosc_message_add_nil (msg);
        break;
			case nOSC_INFTY:
				msg = nosc_message_add_infty (msg);
        break;

			case nOSC_INT64:
				msg = nosc_message_add_int64 (msg, va_arg (args, int64_t));
        break;
			case nOSC_DOUBLE:
				msg = nosc_message_add_double (msg, va_arg (args, double));
        break;
			case nOSC_TIMESTAMP:
			{
				nOSC_Timestamp tt;
				tt.all = va_arg (args, uint64_t);
				msg = nosc_message_add_timestamp (msg, tt);
        break;
			}
			case nOSC_MIDI:
				msg = nosc_message_add_midi (msg, va_arg (args, uint8_t *));
        break;
		}
  va_end (args);


	uint16_t size = nosc_message_serialize (msg, path, buf);
	nosc_message_free (msg);

	return size;
}

void
nosc_message_free (nOSC_Message *msg)
{
	nOSC_Message *ptr = msg;
	while (ptr)
	{
		nOSC_Message *tmp = ptr;
		ptr = ptr->prev;

		switch (tmp->type)
		{
			case nOSC_STRING:
				free (tmp->arg.s);
				break;
			default:
				break;
		}

		free (tmp);
	}
}