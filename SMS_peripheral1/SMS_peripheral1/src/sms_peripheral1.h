
/**
 * \file
 *
 * \brief Startup Template declarations
 *
 * Copyright (c) 2014-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
 * Support</a>
 */

#ifndef __SMS_PERIPHERAL_H__
#define __SMS_PERIPHERAL_H__

/* ------------------------------------------------
 * INCLUDE
 * ------------------------------------------------ */
#include <asf.h>
#include <math.h>
#include "platform.h"
#include "at_ble_api.h"
#include "console_serial.h"
#include "timer_hw.h"
#include "ble_manager.h"
#include "ble_utils.h"
//#include "mpu9250.h"
//#include "log.h"
/* SMS include */
#include "sms_common.h"
#include "sms_button.h"
#include "sms_spi.h"
#include "sms_i2c.h"
#include "sms_sensors.h"
#include "sms_pressure.h"
#include "sms_imu.h"
#include "sms_timer.h"
#include "sms_led.h"
#include "sms_ble.h"


/* ------------------------------------------------
 * MACROS
 * ------------------------------------------------ */
#define SMS_SENDING_WITH_ACK                (false)

#define DBG_PIN_1                           (PIN_LP_GPIO_14) // XPLAINED PRO --> EXT3:11
#define DBG_PIN_2							(PIN_LP_GPIO_15) // XPLAINED PRO --> EXT3:12
#define DBG_PIN_HIGH                        (true)
#define DBG_PIN_LOW                         (false)


/* ------------------------------------------------
 * VARIABLES
 * ------------------------------------------------ */
typedef enum sms_mode {
    SMS_MODE_NONE,
    SMS_MODE_BUTTON_SOLO,
    SMS_MODE_IMU_SOLO,
    SMS_MODE_PRESSURE_SOLO,
    SMS_MODE_BUTTON_IMU,
    SMS_MODE_BUTTON_PRESSURE,
    SMS_MODE_IMU_PRESSURE,
    SMS_MODE_COMPLETE
}sms_mode_t;
volatile sms_mode_t sms_working_mode;

//typedef enum sms_plf_int_src {
    //INT_NONE,
    //INT_BTN0,
    //INT_BTN1,
    //INT_MPU_DRDY,
    //INT_AON_TIMER,
    //INT_DUALTIMER1,
    //INT_DUALTIMER2
//}sms_plf_int_src_t;
//typedef struct sms_plf_int {
    //sms_plf_int_src_t source;
    //volatile bool int_on;
//}sms_plf_int_t;
//volatile sms_plf_int_t sms_current_interrupt;

volatile bool ulp_ready;
volatile bool ulp_active;
volatile bool sensors_active;

volatile uint32_t mypsp, mymsp, myctrl;

/* ------------------------------------------------
 * DECLARATIONS
 * ------------------------------------------------ */
void sms_init_variables(void);

#endif /* __SMS_PERIPHERAL_H__ */
