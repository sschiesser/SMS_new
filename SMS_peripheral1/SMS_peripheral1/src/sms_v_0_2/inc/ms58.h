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
#define MS58_RESET_WAIT_MS             (3)
#define MS58_CONV_WAIT_MS              (999)

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

#define MS58_COEFF_256                 (256)
#define MS58_COEFF_512                 (512)
#define MS58_COEFF_1024                (1024)
#define MS58_COEFF_2048                (2048)
#define MS58_COEFF_4096                (4096)

#define MS58_PROM_VALUES_MAX           (8)
#define MS58_ADC_VALUES_MAX            (2)
#define MS58_BUF_SIZE                  (4)

/* === Types ================================================================ */
typedef enum ms58_datatype_tag {
    MS58_TYPE_PRESS = 0x00,
    MS58_TYPE_TEMP = 0x01
}ms58_datatype_t;

typedef enum sms_ms58_state {
    MS58_STATE_NONE,
    MS58_STATE_RESETTING,
    MS58_STATE_READY,
    MS58_STATE_CONV_PRESSURE,
    MS58_STATE_CONV_TEMPERATURE
}sms_ms58_state_t;

typedef struct ms58_instance {
    bool int_active; // external interrupt from this device enabled
    //bool reset_done; // reset command has been successfully sent
    bool init_ok; // PROM value have been successfully read
    uint8_t osr;
    uint16_t prom_values[MS58_PROM_VALUES_MAX];
    uint32_t adc_values[MS58_ADC_VALUES_MAX];
    int32_t pressure; // calculated pressure value
    int32_t temperature; // calculated temperature value
    bool data_complete;
    sms_ms58_state_t current_state;
    //uint8_t init_retry;
}ms58_instance_t;


#endif