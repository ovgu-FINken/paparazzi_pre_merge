/*
 * Copyright (C) 2014 Andreas Pfohl
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/datalink/mavlink_decoder.h
 *  @brief simple decoder of mavlink message
 */

#ifndef MAVLINK_ENCODER_H
#define MAVLINK_ENCODER_H

#include "std.h"
#include "subsystems/datalink/transport.h"
#include "mcu_periph/uart.h"
#include "modules/datalink/mavlink_decoder.h"

#define MavlinkTransmit(x) TransportLink(MAVLINK_UART, Transmit(x))
#define STXMAV 0xfe

struct mavlink_message_s
{
	uint8_t len;
	uint8_t seq;
	uint8_t sys_id;
	uint8_t comp_id;
	uint8_t msg_id;
	uint8_t *payload;
};

static inline void mavlink_message_send(struct mavlink_message_s *msg)
{
	uint8_t length = msg->len;
	uint8_t *payload = msg->payload;
	uint16_t checksum;
	mavlink_crc_init(&checksum);
	mavlink_crc_accumulate(msg->len, &checksum);
	mavlink_crc_accumulate(msg->seq, &checksum);
	mavlink_crc_accumulate(msg->sys_id, &checksum);
	mavlink_crc_accumulate(msg->comp_id, &checksum);
	mavlink_crc_accumulate(msg->msg_id, &checksum);
	while(length--)
	{
		mavlink_crc_accumulate(*payload++, &checksum);
	}
	mavlink_crc_accumulate(62, &checksum);

  length = msg->len;
	payload = msg->payload;

	MavlinkTransmit(STXMAV);
	MavlinkTransmit(msg->len);
	MavlinkTransmit(msg->seq);
	MavlinkTransmit(msg->sys_id);
	MavlinkTransmit(msg->comp_id);
	MavlinkTransmit(msg->msg_id);
	while(length--)
	{
		MavlinkTransmit(*payload++);
	}
	MavlinkTransmit((uint8_t)(checksum & 0xff));
	MavlinkTransmit((uint8_t)(checksum >> 8));
}

#endif /* MAVLINK_ENCODER_H */
