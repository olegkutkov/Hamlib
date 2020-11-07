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

#ifndef __SAR_H__
#define __SAR_H__

/* System defines */
#define DRIVER_MODEL_NAME "SAR-1"
#define DRIVER_MFG_NAME "SAR"
#define DRIVER_VERSION "20201026.0"
#define DRIVER_LICENCE "LGPL"

#define SERIAL_PORT_RATE_MIN 4800
#define SERIAL_PORT_RATE_MAX 1500000
#define SERIAL_PORT_DATA_BITS 8
#define SERIAL_PORT_STOP_BITS 1
#define SERIAL_PORT_WRITE_DELAY 0
#define SERIAL_PORT_POST_WRITE_DELAY 0
#define SERIAL_PORT_TIMEOUT 1000
#define SERIAL_PORT_RETRY 3

/* Motor and mount params & limits */
#define MOTOR_ID_AZ 2
#define MOTOR_ID_EL 1

#define MOTOR_AZ_SPEED_LO 25
#define MOTOR_EL_SPEED_LO 15

#define MOTOR_DEG_PER_DEC 0.293

#define MOTOR_EL_ZERO_VAL 155
#define MOTOR_AZ_ZERO_VAL 146

#define MOUNT_MIN_AZ 104
#define MOUNT_MAX_AZ 259
#define MOUNT_MIN_EL 0
#define MOUNT_MAX_EL 60

#define MOUNT_AZ_CORRECTION_OFFSET 180
#define MOUNT_EL_CORRECTION_OFFSET 0

/* Dynamixel protocol defines */
#define READ_POLL_RETRY_COUNT 5
#define READ_POLL_TIEMOUT_MS 50

#define HEADER_1   0xFF
#define HEADER_2   0xFF

#define INSTRUCTION_READ    0x02
#define INSTRUCTION_WRITE   0x03

#define CMD_SET_ID           0x03
#define CMD_PING             0x01
#define CMD_GOAL_POSITION    0x1E
#define CMD_GOAL_SPEED       0x20

#define CMD_GOAL_LENGTH      5
#define CMD_POS_LENGTH       4
#define CMD_ID_LENGTH        4
#define CMD_SPEED_LENGTH     5

#define BYTE_PRESENT_POSITION_L  36
#define BYTE_READ_POS       2

#define INSTRUCTION_RETRY_DELAY_US 50000

#define CSUM3(a, b, c) ((~(a + b + c)) & 0xFF)
#define CSUM5(a, b, c, d, e)  ((~(a + b + c + d + e)) & 0xFF)
#define CSUM6(a, b, c, d, e, f)  ((~(a + b + c + d + e + f)) & 0xFF)


#endif

