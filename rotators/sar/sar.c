/*
 *  Hamlib Rotator backend - Small Antenna Rotator Ver 1
 *   Copyright (c) 2020 by Oleg Kutkov <contact@olegkutkov.me>
 *
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <unistd.h>

#include "hamlib/rotator.h"
#include "serial.h"
#include "misc.h"
#include "register.h"

#include "sar.h"

static azimuth_t mount_az_offset = MOUNT_AZ_CORRECTION_OFFSET;
static elevation_t mount_el_offset = MOUNT_EL_CORRECTION_OFFSET;

/* */

static int sar_rot_init(ROT *rot)
{
	/* Nothing to do here, yet */
	return RIG_OK;
}

static int sar_rot_cleanup(ROT *rot)
{
	/* Nothing to do here, yet */
	return RIG_OK;
}

/* Set motor rotation speed */
static int set_ax_speed(hamlib_port_t *port, uint8_t id, uint16_t speed)
{
	uint8_t speed_hi, speed_lo;
	uint8_t packet[9];

	speed_hi = speed >> 8;
	speed_lo = speed;

	packet[0] = HEADER_1;
	packet[1] = HEADER_2;
	packet[2] = id;
	packet[3] = CMD_SPEED_LENGTH;
	packet[4] = INSTRUCTION_WRITE;
	packet[5] = CMD_GOAL_SPEED;
	packet[6] = speed_lo;
	packet[7] = speed_hi;
	packet[8] = CSUM6(id, CMD_SPEED_LENGTH, INSTRUCTION_WRITE, CMD_GOAL_SPEED, speed_lo, speed_hi);

	return write_block(port, (const char *) &packet, sizeof(packet));
}

/* Probe motor ID with ping packet */
static int ax_ping(hamlib_port_t *port, uint8_t id)
{
	uint8_t packet[6];

	packet[0] = HEADER_1;
	packet[1] = HEADER_2;
	packet[2] = id;
	packet[3] = INSTRUCTION_READ;
	packet[4] = CMD_PING;
	packet[5] = CSUM3(id, INSTRUCTION_READ, CMD_PING);

	rig_debug(RIG_DEBUG_VERBOSE, "Probing motor with ID=%d\n", id);

	write_block(port, (const char *) &packet, sizeof(packet));

	if (read_block(port, (char *) &packet, sizeof(packet))  != sizeof(packet)) {
		return -RIG_ETIMEOUT;
	}

	return RIG_OK;
}

static int sar_rot_open(ROT *rot)
{
	int err = ax_ping(&rot->state.rotport, MOTOR_ID_AZ);

	if (err != RIG_OK) {
		rig_debug(RIG_DEBUG_ERR, "Couldn't detect AZ motor with ID=%d\n", MOTOR_ID_AZ);
		return err;
	}

	err = ax_ping(&rot->state.rotport, MOTOR_ID_EL);

	if (err != RIG_OK) {
		rig_debug(RIG_DEBUG_ERR, "Couldn't detect EL motor with ID=%d\n", MOTOR_ID_EL);
		return err;
	}

	set_ax_speed(&rot->state.rotport, MOTOR_ID_AZ, MOTOR_AZ_SPEED_LO);
	set_ax_speed(&rot->state.rotport, MOTOR_ID_EL, MOTOR_EL_SPEED_LO);

	return RIG_OK;
}

/* Move motor with "id" to position "motor_pos" in deg */
static int ax_move(hamlib_port_t *port, uint8_t id, float motor_pos)
{
	uint16_t pos_dec;
	uint8_t pos_hi, pos_lo;
	uint8_t packet[9];

	pos_dec = (uint16_t) (motor_pos / MOTOR_DEG_PER_DEC);

	pos_hi = pos_dec >> 8;
	pos_lo = pos_dec;

	packet[0] = HEADER_1;
	packet[1] = HEADER_2;
	packet[2] = id;
	packet[3] = CMD_GOAL_LENGTH;
	packet[4] = INSTRUCTION_WRITE;
	packet[5] = CMD_GOAL_POSITION;
	packet[6] = pos_lo;
	packet[7] = pos_hi;
	packet[8] = CSUM6(id, CMD_GOAL_LENGTH, INSTRUCTION_WRITE, CMD_GOAL_POSITION, pos_lo, pos_hi);

	return write_block(port, (const char *) &packet, sizeof(packet));
}

/* Simple wrapper around ax_move (sometimes servo is not responding) */
static int try_ax_move(hamlib_port_t *port, uint8_t id, float motor_pos)
{
	int i;

	for (i = 0; i < 5; ++i) {
		if (ax_move(port, id, motor_pos) == RIG_OK) {
			return RIG_OK;
		}

		usleep(INSTRUCTION_RETRY_DELAY_US);
	}

	return -RIG_ETIMEOUT;
}

/* Entry point for the AZ and EL set, values are calibrated */
static int sar_rot_set_position(ROT *rot, azimuth_t az, elevation_t el)
{
	int err;
	azimuth_t az_tmp;
	elevation_t el_tmp = el - mount_el_offset;

	err = try_ax_move(&rot->state.rotport, MOTOR_ID_EL, MOTOR_EL_ZERO_VAL - el_tmp);

	if (err != RIG_OK) {
		return err;
	}

	az_tmp = az - mount_az_offset;

	return try_ax_move(&rot->state.rotport, MOTOR_ID_AZ, MOTOR_AZ_ZERO_VAL - az_tmp);
}

/* Get position of the motor with "id" in deg */
static int ax_get_pos(hamlib_port_t *port, uint8_t id, float *motor_pos)
{
	int err;
	uint16_t servo_dec_pos;
	uint8_t packet[8];

	packet[0] = HEADER_1;
	packet[1] = HEADER_2;
	packet[2] = id;
	packet[3] = CMD_POS_LENGTH;
	packet[4] = INSTRUCTION_READ;
	packet[5] = BYTE_PRESENT_POSITION_L;
	packet[6] = BYTE_READ_POS;
	packet[7] = CSUM5(id, CMD_POS_LENGTH, INSTRUCTION_READ, BYTE_PRESENT_POSITION_L, BYTE_READ_POS);

	err = write_block(port, (const char *) &packet, sizeof(packet));

	if (err != RIG_OK) {
		return err;
	}

	err = read_block(port, (char *) &packet, sizeof(packet));

	if (err != sizeof(packet)) {
		return -RIG_ETRUNC;
	}

	servo_dec_pos = packet[6] << 8;
	servo_dec_pos += packet[5];

	*motor_pos = ((float) servo_dec_pos) * MOTOR_DEG_PER_DEC;

	return RIG_OK;
}

/* Simple wrapper around ax_get_pos (sometimes servo is not responding) */
static int try_ax_get_pos(hamlib_port_t *port, uint8_t id, float *motor_pos)
{
	int i;

	for (i = 0; i < 5; ++i) {
		if (ax_get_pos(port, id, motor_pos) == RIG_OK) {
			return RIG_OK;
		}

		usleep(INSTRUCTION_RETRY_DELAY_US);
	}

	return -RIG_ETIMEOUT;
}

/* Entry point for the AZ and EL get, values are calibrated */
static int sar_rot_get_position(ROT *rot, azimuth_t *az, elevation_t *el)
{
	int err;
	elevation_t el_tmp;
	azimuth_t az_tmp;

	rig_flush(&rot->state.rotport);

	err = try_ax_get_pos(&rot->state.rotport, MOTOR_ID_EL, &el_tmp);

	if (err != RIG_OK) {
		return err;
	}

	*el = MOTOR_EL_ZERO_VAL - el_tmp + mount_el_offset;

	err = try_ax_get_pos(&rot->state.rotport, MOTOR_ID_AZ, &az_tmp);

	if (err != RIG_OK) {
		return err;
	}

	*az = MOTOR_AZ_ZERO_VAL - az_tmp + mount_az_offset;

	return RIG_OK;
}

static int sar_rot_stop(ROT *rot)
{
	/* TODO */
	return RIG_OK;
}

/* Set rotator capabilities */
const struct rot_caps sar_caps =
{
	ROT_MODEL(ROT_MODEL_SAR),
    .model_name =       DRIVER_MODEL_NAME,
    .mfg_name =         DRIVER_MFG_NAME,
    .version =          DRIVER_VERSION,
    .copyright =        DRIVER_LICENCE,
    .status =           RIG_STATUS_UNTESTED,
    .rot_type =         ROT_TYPE_AZEL,
    .port_type =        RIG_PORT_SERIAL,
    .serial_rate_min  = SERIAL_PORT_RATE_MIN,
    .serial_rate_max  = SERIAL_PORT_RATE_MAX,
    .serial_data_bits = SERIAL_PORT_DATA_BITS,
    .serial_stop_bits = SERIAL_PORT_STOP_BITS,
    .serial_parity    = RIG_PARITY_NONE,
    .serial_handshake = RIG_HANDSHAKE_NONE,
    .write_delay      = SERIAL_PORT_WRITE_DELAY,
    .post_write_delay = SERIAL_PORT_POST_WRITE_DELAY,
    .timeout          = SERIAL_PORT_TIMEOUT,
    .retry            = SERIAL_PORT_RETRY,

	.min_az = MOUNT_MIN_AZ,
	.max_az = MOUNT_MAX_AZ,
	.min_el = MOUNT_MIN_EL,
	.max_el = MOUNT_MAX_EL,

	.rot_init = sar_rot_init,
	.rot_cleanup = sar_rot_cleanup,

	.rot_open = sar_rot_open,
	.set_position = sar_rot_set_position,
	.get_position = sar_rot_get_position,
	.stop = sar_rot_stop
};

/* Register rotator */
DECLARE_INITROT_BACKEND(sar)
{
	rig_debug(RIG_DEBUG_VERBOSE, "%s: _init called\n", __func__);

	rot_register(&sar_caps);

	return RIG_OK;
}

