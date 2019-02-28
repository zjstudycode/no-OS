/*
 * libtinyiiod - Tiny IIO Daemon Library
 *
 * Copyright (C) 2016 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#include "tinyiiod-private.h"

#include "compat.h"

struct tinyiiod {
	int32_t instance_id;
	const char *xml;
	const struct tinyiiod_ops *ops;
};

enum {
	READ_BUFFER,
	WRITE_BUFFER,
	BUFFER_NO
};

static char *iiod_buf[BUFFER_NO];

/***************************************************************************//**
 * @brief tinyiiod_create
*******************************************************************************/
struct tinyiiod * tinyiiod_create(const char *xml,
				  const struct tinyiiod_ops *ops)
{
	struct tinyiiod *iiod = malloc(sizeof(*iiod));
	iiod_buf[READ_BUFFER] = (char*)malloc(IIOD_BUFFER_SIZE);
	iiod_buf[WRITE_BUFFER] = (char*)malloc(IIOD_BUFFER_SIZE);

	if (!iiod)
		return NULL;

	iiod->xml = xml;
	iiod->ops = ops;

	return iiod;
}

/***************************************************************************//**
 * @brief tinyiiod_destroy
*******************************************************************************/
void tinyiiod_destroy(struct tinyiiod *iiod)
{
	free(iiod_buf[READ_BUFFER]);
	free(iiod_buf[WRITE_BUFFER]);
	free(iiod);
}

/***************************************************************************//**
 * @brief tinyiiod_read_command
*******************************************************************************/
int32_t tinyiiod_read_command(struct tinyiiod *iiod)
{
	char buf[128];
	int32_t ret;

	ret = tinyiiod_read_line(iiod, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	ret = tinyiiod_parse_string(iiod, buf);
	if (ret < 0)
		tinyiiod_write_value(iiod, ret);

	return ret;
}

/***************************************************************************//**
 * @brief tinyiiod_read_char
*******************************************************************************/
char tinyiiod_read_char(struct tinyiiod *iiod)
{
	char c;

	iiod->ops->read(&iiod->instance_id, &c, 1);
	return c;
}

/***************************************************************************//**
 * @brief tinyiiod_read
*******************************************************************************/
int32_t tinyiiod_read(struct tinyiiod *iiod, char *buf, size_t len)
{
	return iiod->ops->read(&iiod->instance_id, buf, len);
}

/***************************************************************************//**
 * @brief tinyiiod_read_line
*******************************************************************************/
int32_t tinyiiod_read_line(struct tinyiiod *iiod, char *buf, size_t len)
{
	int32_t i;
	bool found = false;

	if(iiod->ops->read_line)
		return iiod->ops->read_line(&iiod->instance_id, buf, len);

	for (i = 0; i < len - 1; i++) {
		buf[i] = tinyiiod_read_char(iiod);

		if (buf[i] != '\n' && buf[i] != '\r')
			found = true;
		else if (found)
			break;
	}

	if (!found || i == len - 1) {
		/* No \n found -> garbage data */
		return -EIO;
	}

	buf[i] = '\0';

	return i;
}

/***************************************************************************//**
 * @brief tinyiiod_write_char
*******************************************************************************/
void tinyiiod_write_char(struct tinyiiod *iiod, char c)
{
	iiod->ops->write(iiod->instance_id, &c, 1);
}

/***************************************************************************//**
 * @brief tinyiiod_write
*******************************************************************************/
void tinyiiod_write(struct tinyiiod *iiod, const char *data, size_t len)
{
	iiod->ops->write(iiod->instance_id, data, len);
}

/***************************************************************************//**
 * @brief tinyiiod_write_string
*******************************************************************************/
void tinyiiod_write_string(struct tinyiiod *iiod, const char *str)
{
	tinyiiod_write(iiod, str, strlen(str));
}

/***************************************************************************//**
 * @brief tinyiiod_write_value
*******************************************************************************/
void tinyiiod_write_value(struct tinyiiod *iiod, int32_t value)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%"PRIi32"\n", value);
	tinyiiod_write_string(iiod, buf);
}

/***************************************************************************//**
 * @brief tinyiiod_write_xml
*******************************************************************************/
void tinyiiod_write_xml(struct tinyiiod *iiod)
{
	size_t len = strlen(iiod->xml);

	tinyiiod_write_value(iiod, len);
	tinyiiod_write(iiod, iiod->xml, len);
	tinyiiod_write_char(iiod, '\n');
}

/***************************************************************************//**
 * @brief tinyiiod_do_read_attr
*******************************************************************************/
void tinyiiod_do_read_attr(struct tinyiiod *iiod, const char *device,
			   const char *channel, bool ch_out, const char *attr, bool debug)
{
	ssize_t ret;

	if (channel)
		ret = iiod->ops->ch_read_attr(device, channel,
					      ch_out, attr, iiod_buf[READ_BUFFER], IIOD_BUFFER_SIZE);
	else
		ret = iiod->ops->read_attr(device, attr,
					   iiod_buf[READ_BUFFER], IIOD_BUFFER_SIZE, debug);

	tinyiiod_write_value(iiod, (int32_t) ret);
	if (ret > 0) {
		iiod_buf[READ_BUFFER][ret] = '\n';
		tinyiiod_write(iiod, iiod_buf[READ_BUFFER], (size_t) ret + 1);
	}
}

/***************************************************************************//**
 * @brief tinyiiod_do_write_attr
*******************************************************************************/
void tinyiiod_do_write_attr(struct tinyiiod *iiod, const char *device,
			    const char *channel, bool ch_out, const char *attr,
			    size_t bytes, bool debug)
{
	ssize_t ret;

	if (bytes > IIOD_BUFFER_SIZE - 1)
		bytes = IIOD_BUFFER_SIZE - 1;

	tinyiiod_read(iiod, iiod_buf[WRITE_BUFFER], bytes);
	iiod_buf[bytes] = '\0';

	if (channel)
		ret = iiod->ops->ch_write_attr(device, channel, ch_out,
					       attr, iiod_buf[WRITE_BUFFER], bytes);
	else
		ret = iiod->ops->write_attr(device, attr, iiod_buf[WRITE_BUFFER], bytes, debug);

	tinyiiod_write_value(iiod, (int32_t) ret);
}

/***************************************************************************//**
 * @brief tinyiiod_do_open
*******************************************************************************/
void tinyiiod_do_open(struct tinyiiod *iiod, const char *device,
		      size_t sample_size, uint32_t mask)
{
	int32_t ret = iiod->ops->open(device, sample_size, mask);
	tinyiiod_write_value(iiod, ret);
}

/***************************************************************************//**
 * @brief tinyiiod_do_close
*******************************************************************************/
void tinyiiod_do_close(struct tinyiiod *iiod, const char *device)
{
	int32_t ret = iiod->ops->close(device);
	tinyiiod_write_value(iiod, ret);
}

/***************************************************************************//**
 * @brief tinyiiod_do_close_instance
*******************************************************************************/
int32_t tinyiiod_do_close_instance(struct tinyiiod *iiod)
{
	return iiod->ops->close_instance(iiod->instance_id);
}

/***************************************************************************//**
 * @brief tinyiiod_do_writebuf
*******************************************************************************/
int32_t tinyiiod_do_writebuf(struct tinyiiod *iiod,
			     const char *device, size_t bytes_count)
{
	int32_t ret;
	char *pbuffer = (char*)malloc(bytes_count);
	if(!pbuffer) {
		ret = -ENOMEM;
		goto err_close;
	}

	tinyiiod_write_value(iiod, (int) bytes_count);
	ret = tinyiiod_read(iiod, pbuffer, bytes_count);
	if(ret < 0)
		goto err_close;
	if(bytes_count != ret) {
		ret = -EPIPE;
		goto err_close;
	}

	ret = iiod->ops->write_data(device, pbuffer, bytes_count);
	if(ret < 0)
		goto err_close;
	tinyiiod_write_value(iiod, (int) bytes_count);

err_close:
	free(pbuffer);
	return ret;
}

/***************************************************************************//**
 * @brief tinyiiod_do_readbuf
*******************************************************************************/
int32_t tinyiiod_do_readbuf(struct tinyiiod *iiod,
			    const char *device, size_t bytes_count)
{
	int32_t ret;
	char *buf = (char*)malloc(bytes_count);
	uint32_t mask;
	char buf_mask[10];

	if(!buf) {
		ret = -ENOMEM;
		tinyiiod_write_value(iiod, ret);
		goto err_close;
	}

	ret = iiod->ops->get_mask(device, &mask);
	if (ret < 0) {
		tinyiiod_write_value(iiod, ret);
		goto err_close;
	}

	ret = (int) iiod->ops->read_data(device, buf, bytes_count);
	if (ret < 0) {
		tinyiiod_write_value(iiod, ret);
		goto err_close;
	}

	tinyiiod_write_value(iiod, ret);
	snprintf(buf_mask, sizeof(buf_mask), "%08"PRIx32"\n", mask);
	tinyiiod_write_string(iiod, buf_mask);
	tinyiiod_write(iiod, buf, (size_t) ret);

err_close:
	free(buf);
	return ret;
}
