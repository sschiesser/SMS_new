/*
* sms_button.c
*
* Created: 03.05.2016 15:02:24
*  Author: Sébastien Schiesser
*/

#include "sms_central1.h"

/************************************************************************/
/* Callback functions --> doing things                                  */
/************************************************************************/
int sms_button_fn(enum sms_btn_ids btn)
{
    return 0;
}
/************************************************************************/
/* Callback functions --> doing things                                  */
/************************************************************************/

/* Initialize gpio for button inputs */
void sms_button_configure_gpio(void)
{
    struct gpio_config config_gpio_pin;

    ///* Button0 @ PIN_AO_GPIO_0 */
    //gpio_get_config_defaults(&config_gpio_pin);
    //config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    //config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    //config_gpio_pin.aon_wakeup = true;
    //if(gpio_pin_set_config(btn0_instance.gpio_pin, &config_gpio_pin) != STATUS_OK) {
        //DBG_LOG_DEV("[sms_button_configure]\tproblem while setting up button0");
    //}
    //
    ///* Button1 @ PIN_AO_GPIO_2 */
    //gpio_get_config_defaults(&config_gpio_pin);
    //config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    //config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    //config_gpio_pin.aon_wakeup = true;
    //if(gpio_pin_set_config(btn1_instance.gpio_pin, &config_gpio_pin) != STATUS_OK) {
        //DBG_LOG("[sms_button_configure]\tProblem while setting up button1");
    //}

    /* Button 0 on SAMB11 XPLAINED */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_NONE;
    if(gpio_pin_set_config(BUTTON_0_PIN, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("[sms_button_configure]\tProblem while setting up XPLAINED user button");
    }
}

/* Register button input callbacks */
void sms_button_register_callbacks(void)
{
    ///* Button0 callback */
    //gpio_register_callback(btn0_instance.gpio_pin, sms_button_bt0_callback, GPIO_CALLBACK_RISING);
    ////gpio_enable_callback(SMS_BTN_0_PIN);
    //
    ///* Button1 callback */
    //gpio_register_callback(btn1_instance.gpio_pin, sms_button_bt1_callback, GPIO_CALLBACK_RISING);
    ////gpio_enable_callback(SMS_BTN_1_PIN);
    
    /* User button callback */
}

/* Disable button input callbacks */
void sms_button_disable_callbacks(void)
{
    ///* Button0 callback */
    //gpio_disable_callback(SMS_BTN_0_PIN);
    ///* Button1 callback */
    //gpio_disable_callback(SMS_BTN_1_PIN);
}

/* Get current buttons state */
enum sms_button_state sms_button_get_state(void)
{
    //bool b0 = gpio_pin_get_input_level(SMS_BTN_0_PIN);
    //bool b1 = gpio_pin_get_input_level(SMS_BTN_1_PIN);
    //DBG_LOG("[sms_button_get_state]\t\tButton state: %d %d", b1, b0);
    //if(b0 && b1) return BUTTON_STATE_BOTH;
    //else if(b0 && !b1) return BUTTON_STATE_B0;
    //else if(!b0 && b1) return BUTTON_STATE_B1;
    //else return BUTTON_STATE_NONE;
    return BUTTON_STATE_NONE;
}

/* En- or disable button interrupts */
void sms_button_toggle_interrupt(enum sms_btn_int_tog tog0, enum sms_btn_int_tog tog1)
{
    //if(tog0 == SMS_BTN_INT_ENABLE) {
        //gpio_enable_callback(btn0_instance.gpio_pin);
    //}
    //else if(tog0 == SMS_BTN_INT_DISABLE) {
        //gpio_disable_callback(btn0_instance.gpio_pin);
    //}
    //
    //if(tog1 == SMS_BTN_INT_ENABLE) {
        //gpio_enable_callback(btn1_instance.gpio_pin);
    //}
    //else if(tog1 == SMS_BTN_INT_DISABLE) {
        //gpio_disable_callback(btn1_instance.gpio_pin);
    //}
}
/* Callbacks --> sending interrupt message to platform */
void sms_button_bt0_callback(void)
{
    //gpio_disable_callback(SMS_BTN_0_PIN);
    //sms_current_interrupt.source = INT_BTN0;
    send_plf_int_msg_ind(btn0_instance.gpio_pin, GPIO_CALLBACK_RISING, NULL, 0);
}
void sms_button_bt1_callback(void)
{
    //gpio_disable_callback(SMS_BTN_1_PIN);
    //sms_current_interrupt.source = INT_BTN1;
    send_plf_int_msg_ind(btn1_instance.gpio_pin, GPIO_CALLBACK_RISING, NULL, 0);
}