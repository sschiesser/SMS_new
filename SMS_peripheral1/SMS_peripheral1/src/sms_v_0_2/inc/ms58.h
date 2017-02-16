/*
* ms58.h
*
* Created: 10.06.2016 15:00:58
*  Author: Sébastien Schiesser
*/
#ifndef MS58_H_
#define MS58_H_

/* === Includes ============================================================= */

/* === Macros =============================================================== */
#define MS58_RESET_WAIT_US             (2800)
#define MS58_CONV_WAIT_US              (8220)
#define MS58_RESET_WAIT_MS             (10) // actually need 3...
#define MS58_CONV_WAIT_MS              (9)

#define MS58_INIT_RETRY_MAX            (4)

#define MS58_ENABLE                    (0)
#define MS58_DISABLE                   (1)

#define MS58_RESET                     (0x1E)
#define MS58_CONV_D1_256               (0x40)
#define	MS58_CONV_D1_512               (0x42)
#define	MS58_CONV_D1_1024              (0x44)
#define	MS58_CONV_D1_2048              (0x46)
#define	MS58_CONV_D1_4096              (0x48)
#define	MS58_CONV_D2_256               (0x50)
#define	MS58_CONV_D2_512               (0x52)
#define	MS58_CONV_D2_1024              (0x54)
#define	MS58_CONV_D2_2048              (0x56)
#define	MS58_CONV_D2_4096              (0x58)
#define	MS58_ADC_READ                  (0x00)
#define	MS58_PROM_READ_0               (0xA0)
#define	MS58_PROM_READ_1               (0xA2)
#define	MS58_PROM_READ_2               (0xA4)
#define	MS58_PROM_READ_3               (0xA6)
#define	MS58_PROM_READ_4               (0xA8)
#define	MS58_PROM_READ_5               (0xAA)
#define	MS58_PROM_READ_6               (0xAC)
#define	MS58_PROM_READ_7               (0xAE)

#define MS58_COEFF_256					(256)
#define MS58_COEFF_512					(512)
#define MS58_COEFF_1024					(1024)
#define MS58_COEFF_2048					(2048)
#define MS58_COEFF_4096					(4096)

#define MS58_PROM_VAL_ERR				(65000)
#define MS58_PROM_VAL_MAX				(8)
#define MS58_ADC_VAL_MAX				(2)
#define MS58_BUF_SIZE					(4)

/* === Types ================================================================ */
typedef enum ms58_datatype_tag {
    MS58_TYPE_PRESS = 0x00,
    MS58_TYPE_TEMP = 0x01
}ms58_datatype_t;

enum ms58_state {
    MS58_STATE_NONE,
    MS58_STATE_RESETTING,
    MS58_STATE_READY,
    MS58_STATE_CONV_PRESSURE,
    MS58_STATE_CONV_TEMPERATURE
};
struct ms58_config_s {
	bool init_ok; // PROM value have been successfully read
    uint8_t osr;
};
struct ms58_output_s {
	uint16_t prom_values[MS58_PROM_VAL_MAX];
	uint32_t adc_values[MS58_ADC_VAL_MAX];
	bool complete;
	int32_t pressure;
	int32_t temperature;
};
struct ms58_interrupt_s {
	bool enabled;
	volatile bool new_value;
	volatile bool rts;
};

#endif