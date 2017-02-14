/*
 * sms_common.c
 *
 * Created: 14.11.2016 12:50:33
 *  Author: Sébastien Schiesser
 */ 

#include "sms_common.h"

/* General functions */
void sms_monitor_configure_gpio(void)
{
    struct gpio_config config_gpio_pin;
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction  = GPIO_PIN_DIR_OUTPUT;
    if(gpio_pin_set_config(DBG_PIN_1, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("Problem while setting gpio pin");
    }
    gpio_pin_set_output_level(DBG_PIN_1, DBG_PIN_LOW);
	
	gpio_get_config_defaults(&config_gpio_pin);
	config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
	if(gpio_pin_set_config(DBG_PIN_2, &config_gpio_pin) != STATUS_OK) {
		DBG_LOG("Problem while setting gpio pin");
	}
	gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_LOW);
}

void sms_monitor_get_states(const char *label)
{
    DBG_LOG_DEV("%s...\t\tB-prev %d, B-cur %d, BLE 0x%02x, T1 %d, T2 %d, SMS %d", label, button_instance.previous_state, button_instance.current_state, ble_instance.current_state, timer1_current_mode, timer2_current_mode, sms_working_mode);
}

void sms_disable_irq(enum sms_irq irq)
{
	if(SMS_IRQ_BTN0 & irq) {
		DBG_LOG_CONT(".");
		NVIC_DisableIRQ(SMS_BTN_0_IRQ);
	}
	if(SMS_IRQ_BTN1 & irq) {
		DBG_LOG_CONT(".");
		NVIC_DisableIRQ(SMS_BTN_1_IRQ);
	}
	if(SMS_IRQ_DRDY & irq) {
		DBG_LOG_CONT(".");
		NVIC_DisableIRQ(SMS_IMU_DRDY_IRQ);
	}
	if(SMS_IRQ_AON_TIMER & irq) {
		DBG_LOG_CONT(".");
		NVIC_DisableIRQ(SMS_TIMER_AON_IRQ);
	}
}

void sms_enable_irq(enum sms_irq irq)
{
	if(SMS_IRQ_BTN0 & irq) {
		DBG_LOG_CONT(".");
		NVIC_ClearPendingIRQ(SMS_BTN_0_IRQ);
		NVIC_EnableIRQ(SMS_BTN_0_IRQ);
	}
	if(SMS_IRQ_BTN1 & irq) {
		DBG_LOG_CONT(".");
		NVIC_ClearPendingIRQ(SMS_BTN_1_IRQ);
		NVIC_EnableIRQ(SMS_BTN_1_IRQ);
	}
	if(SMS_IRQ_DRDY & irq) {
		DBG_LOG_CONT(".");
		NVIC_ClearPendingIRQ(SMS_IMU_DRDY_IRQ);
		NVIC_EnableIRQ(SMS_IMU_DRDY_IRQ);
	}
	if(SMS_IRQ_AON_TIMER & irq) {
		DBG_LOG_CONT(".");
		NVIC_ClearPendingIRQ(SMS_TIMER_AON_IRQ);
		NVIC_EnableIRQ(SMS_TIMER_AON_IRQ);
	}
};