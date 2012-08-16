/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

/**
 *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   kxtf9.h
 *      @brief  Accelerometer setup and handling methods.
*/

#define KXTF9_XOUT_HPF_L                (0x00) /* 0000 0000 */
#define KXTF9_XOUT_HPF_H                (0x01) /* 0000 0001 */
#define KXTF9_YOUT_HPF_L                (0x02) /* 0000 0010 */
#define KXTF9_YOUT_HPF_H                (0x03) /* 0000 0011 */
#define KXTF9_ZOUT_HPF_L                (0x04) /* 0001 0100 */
#define KXTF9_ZOUT_HPF_H                (0x05) /* 0001 0101 */
#define KXTF9_XOUT_L                    (0x06) /* 0000 0110 */
#define KXTF9_XOUT_H                    (0x07) /* 0000 0111 */
#define KXTF9_YOUT_L                    (0x08) /* 0000 1000 */
#define KXTF9_YOUT_H                    (0x09) /* 0000 1001 */
#define KXTF9_ZOUT_L                    (0x0A) /* 0001 1010 */
#define KXTF9_ZOUT_H                    (0x0B) /* 0001 1011 */
#define KXTF9_ST_RESP                   (0x0C) /* 0000 1100 */
#define KXTF9_WHO_AM_I                  (0x0F) /* 0000 1111 */
#define KXTF9_TILT_POS_CUR              (0x10) /* 0001 0000 */
#define KXTF9_TILT_POS_PRE              (0x11) /* 0001 0001 */
#define KXTF9_INT_SRC_REG1            (0x15) /* 0001 0101 */
#define KXTF9_INT_SRC_REG2              (0x16) /* 0001 0110 */
#define KXTF9_STATUS_REG                (0x18) /* 0001 1000 */
#define KXTF9_INT_REL                   (0x1A) /* 0001 1010 */
#define KXTF9_CTRL_REG1                 (0x1B) /* 0001 1011 */
#define KXTF9_CTRL_REG2                 (0x1C) /* 0001 1100 */
#define KXTF9_CTRL_REG3                 (0x1D) /* 0001 1101 */
#define KXTF9_INT_CTRL_REG1             (0x1E) /* 0001 1110 */
#define KXTF9_INT_CTRL_REG2             (0x1F) /* 0001 1111 */
#define KXTF9_INT_CTRL_REG3           (0x20) /* 0010 0000 */
#define KXTF9_DATA_CTRL_REG             (0x21) /* 0010 0001 */
#define KXTF9_TILT_TIMER                (0x28) /* 0010 1000 */
#define KXTF9_WUF_TIMER                 (0x29) /* 0010 1001 */
#define KXTF9_TDT_TIMER                 (0x2B) /* 0010 1011 */
#define KXTF9_TDT_H_THRESH              (0x2C) /* 0010 1100 */
#define KXTF9_TDT_L_THRESH              (0x2D) /* 0010 1101 */
#define KXTF9_TDT_TAP_TIMER             (0x2E) /* 0010 1110 */
#define KXTF9_TDT_TOTAL_TIMER           (0x2F) /* 0010 1111 */
#define KXTF9_TDT_LATENCY_TIMER         (0x30) /* 0011 0000 */
#define KXTF9_TDT_WINDOW_TIMER          (0x31) /* 0011 0001 */
#define KXTF9_WUF_THRESH                (0x5A) /* 0101 1010 */
#define KXTF9_TILT_ANGLE                (0x5C) /* 0101 1100 */
#define KXTF9_HYST_SET                  (0x5F) /* 0101 1111 */

#define KXTF9_MAX_DUR (0xFF)
#define KXTF9_MAX_THS (0xFF)
#define KXTF9_THS_COUNTS_P_G (32)

extern struct acc_data cal_data;
